/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : curtis_io.c
 * @brief          : Read and drive the interface signals to/from the Curtis 1510.
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "curtis_io.h"
#include "main.h"     /* pin defines + HAL_GPIO_ReadPin / HAL_GetTick */
#include "i2c.h"      /* hi2c2 for the MCP4725 MCOR DAC */

/* Private define ------------------------------------------------------------*/
#define ADC_VREF              5.0f
#define ADC_FULL_SCALE        4095.0f

/* MCP4725 MCOR DAC on I2C2 (PA8=SDA, PA9=SCL). Default 7-bit address 0x60
 * (A0 = GND); change to 0x61 if the board straps A0 high, or per the variant. */
#define MCP4725_I2C_ADDR      0x60u
/* Supply/reference the MCP4725 runs from; sets the full-scale output voltage.
 * Confirm against the board — a Curtis MCOR throttle is typically 0..5 V. */
#define MCP4725_VREF          5.0f
#define MCP4725_MAX_CODE      4095u
#define MCP4725_I2C_TIMEOUT   10u   /* ms */

/* TIM2 counts at the APB1 timer clock. SYSCLK = 170 MHz and APB1 prescaler = 1,
 * so the TIM2 kernel clock is the full 170 MHz; MX_TIM2_Init() runs it with
 * prescaler 0, i.e. one tick = 1/170 MHz. */
#define CURTIS_SPEED_TIMER_HZ   170000000UL
/* Treat the wheel as stopped if no capture edge arrives within this window, so a
 * stale frequency does not linger after the pulses stop. */
#define CURTIS_SPEED_TIMEOUT_MS 500u

/* Speed-sensor scaling: 50 pulses per wheel revolution, 1.57 m circumference.
 *   rev/s = Hz / PPR ;  m/s = rev/s * circumference                          */
#define CURTIS_SPEED_PPR          50.0f
#define CURTIS_WHEEL_CIRCUM_M     1.57f

/* Private variables ---------------------------------------------------------*/
/* Speed-capture state, written by the ISR (via CurtisIO_SpeedOnCapture) and read
 * by CurtisIO_SpeedHz in the main loop. */
static volatile uint32_t s_last_capture = 0;
static volatile uint8_t  s_have_prev    = 0;
static volatile uint32_t s_last_edge_ms = 0;
static volatile float    s_freq_hz      = 0.0f;

/* Last MCOR DAC code written (any driver), for /speed_diagnostics readback. */
static volatile uint16_t s_last_mcor_code = 0;

/* Exported functions --------------------------------------------------------*/
uint8_t CurtisIO_Forward(void) {
    return (HAL_GPIO_ReadPin(Forward_IN_to_MCU_GPIO_Port, Forward_IN_to_MCU_Pin)
            == GPIO_PIN_SET) ? 1u : 0u;
}

uint8_t CurtisIO_Backward(void) {
    return (HAL_GPIO_ReadPin(Backward_IN_to_MCU_GPIO_Port, Backward_IN_to_MCU_Pin)
            == GPIO_PIN_SET) ? 1u : 0u;
}

uint8_t CurtisIO_Pedal(void) {
    return (HAL_GPIO_ReadPin(Pedal_IN_to_MCU_GPIO_Port, Pedal_IN_to_MCU_Pin)
            == GPIO_PIN_SET) ? 1u : 0u;
}

void CurtisIO_ReadDigital(CurtisDigitalInputs_t *out) {
    out->forward  = CurtisIO_Forward();
    out->backward = CurtisIO_Backward();
    out->pedal    = CurtisIO_Pedal();
}

float CurtisIO_McorVolts(uint16_t mcor_adc_raw) {
    return ((float) mcor_adc_raw / ADC_FULL_SCALE) * ADC_VREF;
}

void CurtisIO_SpeedOnCapture(uint32_t capture) {
    if (s_have_prev) {
        /* 32-bit unsigned subtraction wraps correctly across the timer's rollover. */
        uint32_t delta = capture - s_last_capture;
        if (delta != 0u) {
            s_freq_hz = (float) CURTIS_SPEED_TIMER_HZ / (float) delta;
        }
    }
    s_last_capture = capture;
    s_have_prev    = 1u;
    s_last_edge_ms = HAL_GetTick();
}

float CurtisIO_SpeedHz(void) {
    if (!s_have_prev ||
        (HAL_GetTick() - s_last_edge_ms) > CURTIS_SPEED_TIMEOUT_MS) {
        return 0.0f;
    }
    return s_freq_hz;
}

float CurtisIO_SpeedMps(void) {
    return (CurtisIO_SpeedHz() / CURTIS_SPEED_PPR) * CURTIS_WHEEL_CIRCUM_M;
}

/* ------------------------------- Output side ------------------------------ */
void CurtisIO_OutputEnable(uint8_t on) {
    HAL_GPIO_WritePin(Relay_Mode_GPIO_Port, Relay_Mode_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void CurtisIO_SetForward(uint8_t on) {
    HAL_GPIO_WritePin(Forward_IN_from_MCU_GPIO_Port, Forward_IN_from_MCU_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void CurtisIO_SetBackward(uint8_t on) {
    HAL_GPIO_WritePin(Backward_IN_from_MCU_GPIO_Port, Backward_IN_from_MCU_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void CurtisIO_SetPedal(uint8_t on) {
    HAL_GPIO_WritePin(Pedal_IN_from_MCU_GPIO_Port, Pedal_IN_from_MCU_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void CurtisIO_WriteDigitalOut(uint8_t forward, uint8_t backward, uint8_t pedal) {
    CurtisIO_SetForward(forward);
    CurtisIO_SetBackward(backward);
    CurtisIO_SetPedal(pedal);
}

uint8_t CurtisIO_McorWriteRaw(uint16_t code12) {
    if (code12 > MCP4725_MAX_CODE) {
        code12 = MCP4725_MAX_CODE;
    }
    /* MCP4725 fast-write: 2 bytes.
     *   byte0 = [C2 C1 PD1 PD0 D11 D10 D9 D8] with C=00 (fast write), PD=00 (on)
     *   byte1 = [D7 .. D0] */
    uint8_t buf[2];
    buf[0] = (uint8_t) ((code12 >> 8) & 0x0Fu);
    buf[1] = (uint8_t) (code12 & 0xFFu);
    s_last_mcor_code = code12;   /* remember for CurtisIO_McorOutVolts() */
    return (HAL_I2C_Master_Transmit(&hi2c2, (uint16_t) (MCP4725_I2C_ADDR << 1),
            buf, sizeof(buf), MCP4725_I2C_TIMEOUT) == HAL_OK) ? 1u : 0u;
}

uint8_t CurtisIO_McorWriteVolts(float volts) {
    if (volts < 0.0f)         volts = 0.0f;
    if (volts > MCP4725_VREF) volts = MCP4725_VREF;
    uint16_t code = (uint16_t) ((volts / MCP4725_VREF) * (float) MCP4725_MAX_CODE + 0.5f);
    return CurtisIO_McorWriteRaw(code);
}

/* ----------------------- Output-side state readback ----------------------- */
uint8_t CurtisIO_ModeRelayActive(void) {
    return (HAL_GPIO_ReadPin(Relay_Mode_GPIO_Port, Relay_Mode_Pin)
            == GPIO_PIN_SET) ? 1u : 0u;
}

uint8_t CurtisIO_GetOutForward(void) {
    return (HAL_GPIO_ReadPin(Forward_IN_from_MCU_GPIO_Port, Forward_IN_from_MCU_Pin)
            == GPIO_PIN_SET) ? 1u : 0u;
}

uint8_t CurtisIO_GetOutBackward(void) {
    return (HAL_GPIO_ReadPin(Backward_IN_from_MCU_GPIO_Port, Backward_IN_from_MCU_Pin)
            == GPIO_PIN_SET) ? 1u : 0u;
}

uint8_t CurtisIO_GetOutPedal(void) {
    return (HAL_GPIO_ReadPin(Pedal_IN_from_MCU_GPIO_Port, Pedal_IN_from_MCU_Pin)
            == GPIO_PIN_SET) ? 1u : 0u;
}

float CurtisIO_McorOutVolts(void) {
    return ((float) s_last_mcor_code / (float) MCP4725_MAX_CODE) * MCP4725_VREF;
}

/* ---------------------------- Output self-test ---------------------------- */
/* MCOR triangle sweep: code moves by this much each step; a full 0..4095 ramp
 * therefore takes ~4095/64 ≈ 64 steps (~6.4 s at the default 100 ms period),
 * slow enough to watch the wiper voltage climb and fall. */
#define OUTTEST_MCOR_STEP   64

/* Live-Expression telemetry (see curtis_io.h for meanings). */
volatile uint8_t  CurtisIO_OutTest_forward    = 0;
volatile uint8_t  CurtisIO_OutTest_backward   = 0;
volatile uint8_t  CurtisIO_OutTest_pedal      = 0;
volatile uint8_t  CurtisIO_OutTest_phase      = 0;
volatile uint16_t CurtisIO_OutTest_mcor_code  = 0;
volatile float    CurtisIO_OutTest_mcor_volts = 0.0f;
volatile uint8_t  CurtisIO_OutTest_i2c_ok     = 0;

void CurtisIO_OutputTestRun(uint8_t enable, uint32_t period_ms) {
    static uint8_t  running   = 0;   /* de-glitched run state (edge detect) */
    static uint32_t last_tick = 0;   /* HAL tick of the last pattern advance */
    static uint16_t code      = 0;   /* current MCOR DAC code */
    static int16_t  step      = OUTTEST_MCOR_STEP;  /* sweep direction/size */
    static uint8_t  phase     = 0;   /* digital walk phase 0..3 */

    uint32_t now = HAL_GetTick();

    if (!enable) {
        /* Falling edge: park everything and hand control back to the input side.
         * Idle after that (nothing to do until re-enabled). */
        if (running) {
            CurtisIO_WriteDigitalOut(0, 0, 0);
            CurtisIO_McorWriteRaw(0);
            CurtisIO_OutputEnable(0);
            CurtisIO_OutTest_forward    = 0;
            CurtisIO_OutTest_backward   = 0;
            CurtisIO_OutTest_pedal      = 0;
            CurtisIO_OutTest_phase      = 0;
            CurtisIO_OutTest_mcor_code  = 0;
            CurtisIO_OutTest_mcor_volts = 0.0f;
            code    = 0;
            step    = OUTTEST_MCOR_STEP;
            phase   = 0;
            running = 0;
        }
        return;
    }

    if (!running) {
        /* Rising edge: route the MCU outputs to the Curtis (mode relay ON) and
         * let the relay settle one period before we start driving. */
        CurtisIO_OutputEnable(1);
        running   = 1;
        last_tick = now;
        return;
    }

    if (period_ms == 0u) {
        period_ms = 100u;
    }
    if ((now - last_tick) < period_ms) {
        return;   /* not time for the next step yet */
    }
    last_tick = now;

    /* Digital outputs: light one line at a time (F -> B -> Pedal -> all off),
     * so each output visibly toggles in turn. */
    uint8_t f = (uint8_t) (phase == 0u);
    uint8_t b = (uint8_t) (phase == 1u);
    uint8_t p = (uint8_t) (phase == 2u);
    CurtisIO_WriteDigitalOut(f, b, p);
    CurtisIO_OutTest_forward  = f;
    CurtisIO_OutTest_backward = b;
    CurtisIO_OutTest_pedal    = p;
    CurtisIO_OutTest_phase    = phase;
    phase = (uint8_t) ((phase + 1u) & 0x03u);   /* 0,1,2,3(all off), wrap */

    /* MCOR DAC: triangle sweep 0 -> MAX -> 0 so the wiper voltage ramps up and
     * down continuously. Bounce the direction at each end. */
    int32_t next = (int32_t) code + step;
    if (next >= (int32_t) MCP4725_MAX_CODE) {
        next = (int32_t) MCP4725_MAX_CODE;
        step = (int16_t) -step;
    } else if (next <= 0) {
        next = 0;
        step = (int16_t) -step;
    }
    code = (uint16_t) next;

    CurtisIO_OutTest_i2c_ok     = CurtisIO_McorWriteRaw(code);
    CurtisIO_OutTest_mcor_code  = code;
    CurtisIO_OutTest_mcor_volts = ((float) code / (float) MCP4725_MAX_CODE) * MCP4725_VREF;
}

/* -------------------------- Manual output override ------------------------ */
/* Live-Expression override values: edit these, then set CurtisIO_Override_enable
 * (in main.c) to drive the Curtis outputs to exactly these values instead of the
 * rotating self-test pattern above. */
volatile uint8_t CurtisIO_Override_forward    = 0;   /* Forward  out line, 0/1 */
volatile uint8_t CurtisIO_Override_backward   = 0;   /* Backward out line, 0/1 */
volatile uint8_t CurtisIO_Override_pedal      = 0;   /* Pedal    out line, 0/1 */
volatile float   CurtisIO_Override_mcor_volts = 0.0f;/* MCOR DAC target, 0..VREF */
volatile uint8_t CurtisIO_Override_i2c_ok     = 0;   /* 1 = last DAC write ACKed */

void CurtisIO_OutputOverrideRun(uint8_t enable) {
    static uint8_t running    = 0;      /* de-glitched run state (edge detect) */
    static float   last_volts = -1.0f;  /* last value actually written to the DAC */

    if (!enable) {
        /* Falling edge: park every output and hand control back to the input
         * side. Idle after that until re-enabled. */
        if (running) {
            CurtisIO_WriteDigitalOut(0, 0, 0);
            CurtisIO_McorWriteRaw(0);
            CurtisIO_OutputEnable(0);
            CurtisIO_Override_i2c_ok = 0;
            last_volts = -1.0f;
            running    = 0;
        }
        return;
    }

    if (!running) {
        /* Rising edge: route the MCU outputs to the Curtis (mode relay ON) and
         * let the relay settle one pass before driving. */
        CurtisIO_OutputEnable(1);
        running    = 1;
        last_volts = -1.0f;   /* force the DAC to be (re)written on the next pass */
        return;
    }

    /* Digital lines: cheap and idempotent, so apply live every pass. */
    CurtisIO_WriteDigitalOut(CurtisIO_Override_forward,
                             CurtisIO_Override_backward,
                             CurtisIO_Override_pedal);

    /* MCOR DAC: only hit I2C when the requested voltage changed, so a fast main
     * loop does not saturate I2C2 with identical writes. On a NAK, leave
     * last_volts stale so the write is retried next pass. */
    float v = CurtisIO_Override_mcor_volts;
    if (v != last_volts) {
        CurtisIO_Override_i2c_ok = CurtisIO_McorWriteVolts(v);
        if (CurtisIO_Override_i2c_ok) {
            last_volts = v;
        }
    }
}

/* ----------------------- Closed-loop speed control ------------------------ */
/* PID update period. The loop only recomputes the throttle (and rewrites the
 * DAC) this often, which fixes dt for the I/D terms and keeps a fast main loop
 * from saturating I2C2 with identical writes. */
#define SPEED_CTRL_PERIOD_MS    20u
#define SPEED_CTRL_DT           (SPEED_CTRL_PERIOD_MS / 1000.0f)   /* seconds */

/* Below this |target| (m/s) the command is treated as "stop": all lines low and
 * the DAC parked at 0, integrator cleared. */
#define SPEED_CTRL_DEADBAND_MPS 0.01f

/* MCOR throttle output limits. The PID output (volts) is clamped here; it also
 * bounds the integrator (anti-windup) so it cannot charge past what the DAC can
 * deliver. Confirm the upper limit against the Curtis MCOR active range. */
#define SPEED_CTRL_MCOR_MIN_V   0.0f
#define SPEED_CTRL_MCOR_MAX_V   5.0f

/* Default PID gains (error m/s -> volts). Start conservative; tune live. */
volatile float CurtisIO_Speed_Kp = 1.0f;
volatile float CurtisIO_Speed_Ki = 0.1f;
volatile float CurtisIO_Speed_Kd = 0.0f;

/* Live-Expression telemetry (see curtis_io.h for meanings). */
volatile float   CurtisIO_Speed_target_mps   = 0.0f;
volatile float   CurtisIO_Speed_measured_mps = 0.0f;
volatile float   CurtisIO_Speed_error        = 0.0f;
volatile float   CurtisIO_Speed_mcor_volts   = 0.0f;
volatile uint8_t CurtisIO_Speed_forward      = 0;
volatile uint8_t CurtisIO_Speed_backward     = 0;
volatile uint8_t CurtisIO_Speed_pedal        = 0;
volatile uint8_t CurtisIO_Speed_i2c_ok       = 0;

void CurtisIO_SpeedControlRun(uint8_t enable, float target_mps) {
    static uint8_t  running   = 0;      /* de-glitched run state (edge detect) */
    static uint32_t last_tick = 0;      /* HAL tick of the last PID update */
    static float    integ     = 0.0f;   /* PID integral accumulator (volts) */
    static float    prev_err  = 0.0f;   /* previous error, for the derivative */

    if (!enable) {
        /* Falling edge: park every output, clear the loop, and hand control back
         * to the input side. Idle after that until re-enabled. */
        if (running) {
            CurtisIO_WriteDigitalOut(0, 0, 0);
            CurtisIO_McorWriteRaw(0);
            CurtisIO_OutputEnable(0);
            CurtisIO_Speed_forward    = 0;
            CurtisIO_Speed_backward   = 0;
            CurtisIO_Speed_pedal      = 0;
            CurtisIO_Speed_mcor_volts = 0.0f;
            CurtisIO_Speed_error      = 0.0f;
            CurtisIO_Speed_i2c_ok     = 0;
            integ    = 0.0f;
            prev_err = 0.0f;
            running  = 0;
        }
        return;
    }

    uint32_t now = HAL_GetTick();

    if (!running) {
        /* Rising edge: route the MCU outputs to the Curtis (mode relay ON) and
         * let the relay settle one period before we start driving. */
        CurtisIO_OutputEnable(1);
        integ     = 0.0f;
        prev_err  = 0.0f;
        running   = 1;
        last_tick = now;
        return;
    }

    if ((now - last_tick) < SPEED_CTRL_PERIOD_MS) {
        return;   /* not time for the next PID update yet */
    }
    last_tick = now;

    CurtisIO_Speed_target_mps = target_mps;
    float measured = CurtisIO_SpeedMps();          /* magnitude, m/s */
    CurtisIO_Speed_measured_mps = measured;

    /* Stop command: below the deadband we cut every line and zero the DAC. Reset
     * the integrator so it does not wind up while parked. */
    if (target_mps > -SPEED_CTRL_DEADBAND_MPS && target_mps < SPEED_CTRL_DEADBAND_MPS) {
        CurtisIO_WriteDigitalOut(0, 0, 0);
        CurtisIO_Speed_i2c_ok     = CurtisIO_McorWriteRaw(0);
        CurtisIO_Speed_forward    = 0;
        CurtisIO_Speed_backward   = 0;
        CurtisIO_Speed_pedal      = 0;
        CurtisIO_Speed_mcor_volts = 0.0f;
        CurtisIO_Speed_error      = 0.0f;
        integ    = 0.0f;
        prev_err = 0.0f;
        return;
    }

    /* Direction from the sign of the target; pedal high whenever we drive. */
    uint8_t forward  = (target_mps > 0.0f) ? 1u : 0u;
    uint8_t backward = (target_mps > 0.0f) ? 0u : 1u;
    CurtisIO_WriteDigitalOut(forward, backward, 1u);
    CurtisIO_Speed_forward  = forward;
    CurtisIO_Speed_backward = backward;
    CurtisIO_Speed_pedal    = 1u;

    /* PID on the magnitude: regulate |target| against the measured magnitude
     * (the sensor is unsigned, and the direction lines already point the wheel
     * the right way). */
    float sp    = (target_mps > 0.0f) ? target_mps : -target_mps;   /* |target| */
    float error = sp - measured;

    /* Trial integral with anti-windup: only keep the new accumulation if the
     * resulting output has not saturated, so the I-term cannot wind past the
     * DAC's reach. */
    float integ_next = integ + error * SPEED_CTRL_DT;
    float deriv      = (error - prev_err) / SPEED_CTRL_DT;
    float output = CurtisIO_Speed_Kp * error
                 + CurtisIO_Speed_Ki * integ_next
                 + CurtisIO_Speed_Kd * deriv;

    if (output > SPEED_CTRL_MCOR_MAX_V) {
        output = SPEED_CTRL_MCOR_MAX_V;          /* saturated high: hold integral */
    } else if (output < SPEED_CTRL_MCOR_MIN_V) {
        output = SPEED_CTRL_MCOR_MIN_V;          /* saturated low: hold integral */
    } else {
        integ = integ_next;                      /* in range: accept accumulation */
    }
    prev_err = error;

    CurtisIO_Speed_error      = error;
    CurtisIO_Speed_mcor_volts = output;
    CurtisIO_Speed_i2c_ok     = CurtisIO_McorWriteVolts(output);
}

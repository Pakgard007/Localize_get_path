/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : curtis_io.h
 * @brief          : Read and drive the interface signals to/from the Curtis 1510.
 ******************************************************************************
 * @attention
 *
 * Groups the three ways this shield senses the Curtis 1510:
 *   - Digital direction/pedal lines (GPIO input)      : Forward/Backward/Pedal
 *   - MCOR throttle wiper (analog, ADC1_IN2 on PA1)   : CurtisIO_McorVolts()
 *   - Motor speed sensor pulse train (TIM2 CH1 input  : CurtisIO_SpeedHz()
 *     capture on PA15)
 *
 * Pure sensing only: the reads talk to the STM32 GPIO/ADC/timer and return
 * their result. Pin identities come from main.h. The speed path keeps its own
 * capture state, fed from the TIM2 input-capture ISR.
 *
 * NOTE on polarity: the digital lines are wired GPIO_NOPULL (driven by the
 * Curtis), so the reads return the raw pin level (1 = high). Confirm the active
 * sense against the 1510 wiring at the call site.
 ******************************************************************************
 */
/* USER CODE END Header */

#ifndef INC_CURTIS_IO_H_
#define INC_CURTIS_IO_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
/* Snapshot of the three digital lines from the Curtis (raw levels, 1 = high). */
typedef struct {
    uint8_t forward;    /* Forward_IN_to_MCU  (PC5) */
    uint8_t backward;   /* Backward_IN_to_MCU (PC4) */
    uint8_t pedal;      /* Pedal_IN_to_MCU    (PB0) */
} CurtisDigitalInputs_t;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Read all three digital direction/pedal lines at once.
  * @param  out : receives the raw pin levels (1 = high).
  */
void CurtisIO_ReadDigital(CurtisDigitalInputs_t *out);

/* Individual raw reads (1 = pin high, 0 = pin low). */
uint8_t CurtisIO_Forward(void);
uint8_t CurtisIO_Backward(void);
uint8_t CurtisIO_Pedal(void);

/**
  * @brief  Convert an MCOR throttle ADC sample (ADC1_IN2 / PA1) to volts.
  * @param  mcor_adc_raw : raw 12-bit DMA sample for the MCOR rank.
  * @retval Wiper voltage, 0..VREF.
  */
float CurtisIO_McorVolts(uint16_t mcor_adc_raw);

/**
  * @brief  Feed one TIM2 CH1 capture value. Call from the input-capture ISR.
  * @param  capture : the 32-bit counter value latched on the edge.
  */
void CurtisIO_SpeedOnCapture(uint32_t capture);

/**
  * @brief  Latest measured speed-sensor pulse frequency.
  * @retval Frequency in Hz, or 0 if no edge has arrived recently (stopped).
  */
float CurtisIO_SpeedHz(void);

/**
  * @brief  Latest wheel speed converted to linear velocity.
  * @retval Speed magnitude in m/s (always >= 0 — the pulse sensor cannot tell
  *         direction). Uses 50 pulses/rev and a 1.57 m wheel circumference:
  *             m/s = (Hz / 50) * 1.57
  */
float CurtisIO_SpeedMps(void);

/* ============================= Output side ================================ *
 * Drive the Curtis 1510 instead of sensing it. The mode relay (Relay_Mode /
 * PA10) selects which side is wired through, so it MUST be energised before the
 * MCU's outputs reach the Curtis: call CurtisIO_OutputEnable(1) first, then use
 * the setters / DAC below; CurtisIO_OutputEnable(0) hands control back to the
 * pass-through (input) side. The setters do not toggle the relay themselves —
 * the mode relay needs settling time, so switching sides is left explicit.
 * ------------------------------------------------------------------------- */

/**
  * @brief  Energise/de-energise the mode relay (Relay_Mode, PA10).
  * @param  on : 1 = route the MCU outputs to the Curtis (output side active),
  *              0 = release back to the pass-through / input side.
  */
void CurtisIO_OutputEnable(uint8_t on);

/* Drive the direction/pedal output lines to the Curtis (Forward/Backward/Pedal
 * _IN_from_MCU, PC6/PC8/PC7). 1 = high, 0 = low. Requires OutputEnable(1). */
void CurtisIO_SetForward(uint8_t on);
void CurtisIO_SetBackward(uint8_t on);
void CurtisIO_SetPedal(uint8_t on);
void CurtisIO_WriteDigitalOut(uint8_t forward, uint8_t backward, uint8_t pedal);

/**
  * @brief  Write the MCOR throttle output via the MCP4725 DAC (I2C2, PA8/PA9).
  * @param  code12 : 12-bit DAC code, 0..4095 (clamped). Uses MCP4725 fast-write
  *                  mode (RAM only — no EEPROM wear on frequent updates).
  * @retval 1 if the I2C transfer succeeded, 0 otherwise.
  * @note   Requires OutputEnable(1) for the Curtis to see the level.
  */
uint8_t CurtisIO_McorWriteRaw(uint16_t code12);

/**
  * @brief  Write the MCOR output as a voltage (0..MCP4725 VREF).
  * @param  volts : target wiper voltage; clamped to [0, VREF].
  * @retval 1 if the I2C transfer succeeded, 0 otherwise.
  */
uint8_t CurtisIO_McorWriteVolts(float volts);

/* ----------------------- Output-side state readback ----------------------- *
 * Read back what the MCU is currently driving to the Curtis. Used to fill the
 * /speed_diagnostics CAN frame (0x123). These reflect the last levels/values
 * written by any of the output drivers (speed control / override / self-test).
 * ------------------------------------------------------------------------- */

/**
  * @brief  Mode-relay state (Relay_Mode, PA10): 1 = MCU outputs routed to the
  *         Curtis (output side active), 0 = pass-through / input side.
  */
uint8_t CurtisIO_ModeRelayActive(void);

/* Current output-line levels driven to the Curtis (readback of the ODR on the
 * Forward/Backward/Pedal _IN_from_MCU pins). 1 = high, 0 = low. */
uint8_t CurtisIO_GetOutForward(void);
uint8_t CurtisIO_GetOutBackward(void);
uint8_t CurtisIO_GetOutPedal(void);

/**
  * @brief  Last MCOR throttle voltage written to the MCP4725 DAC.
  * @retval Volts (0..VREF), tracked from every CurtisIO_McorWrite*() call.
  */
float CurtisIO_McorOutVolts(void);

/* ============================ Output self-test ============================ *
 * Bench test that exercises every output at once so you can watch them move in
 * a Live Expression (and on a scope/meter): it energises the mode relay, walks
 * the Forward/Backward/Pedal output lines through a rotating pattern, and sweeps
 * the MCOR DAC up and down as a triangle. Non-blocking — paces itself off
 * HAL_GetTick — so call it every main-loop pass.
 *
 * Drive it from a single enable flag you flip in a Live Expression:
 *   - enable 0 -> 1 (rising edge)  : mode relay ON, start driving/sweeping
 *   - enable 1 -> 0 (falling edge) : zero every output, DAC = 0, mode relay OFF
 *     (control handed back to the pass-through / input side).
 * Watch the CurtisIO_OutTest_* variables below to see the values step.
 * ------------------------------------------------------------------------- */

/**
  * @brief  One step of the output self-test. Call every main-loop pass.
  * @param  enable    : 1 = run the test, 0 = idle (releases outputs on the edge).
  * @param  period_ms : ms between advancing the pattern (0 -> 100 ms default).
  */
void CurtisIO_OutputTestRun(uint8_t enable, uint32_t period_ms);

/* Live-Expression telemetry, refreshed each test step. */
extern volatile uint8_t  CurtisIO_OutTest_forward;    /* last Forward  out level */
extern volatile uint8_t  CurtisIO_OutTest_backward;   /* last Backward out level */
extern volatile uint8_t  CurtisIO_OutTest_pedal;      /* last Pedal    out level */
extern volatile uint8_t  CurtisIO_OutTest_phase;      /* digital walk phase 0..3 */
extern volatile uint16_t CurtisIO_OutTest_mcor_code;  /* last DAC code, 0..4095  */
extern volatile float    CurtisIO_OutTest_mcor_volts; /* that code as volts      */
extern volatile uint8_t  CurtisIO_OutTest_i2c_ok;     /* 1 = last DAC write ACKed */

/* ========================= Manual output override ========================= *
 * Like the self-test above, but you set each output value yourself in a Live
 * Expression instead of watching an automatic pattern. Set the CurtisIO_Override_*
 * values below, then flip the enable flag (output_override_enable in main.c) to 1:
 *   - enable 0 -> 1 : mode relay ON, outputs driven to the override values
 *   - enable 1 -> 0 : zero every output, DAC = 0, mode relay OFF (input side)
 * While enabled the values are applied live, so you can change Forward/Backward/
 * Pedal/MCOR on the fly. Non-blocking — call it every main-loop pass. Do not run
 * this and CurtisIO_OutputTestRun() at the same time (they share the outputs).
 * ------------------------------------------------------------------------- */

/**
  * @brief  Apply the manual output override. Call every main-loop pass.
  * @param  enable : 1 = drive the outputs to the CurtisIO_Override_* values,
  *                  0 = idle (releases outputs on the falling edge).
  */
void CurtisIO_OutputOverrideRun(uint8_t enable);

/* Override values — edit these in a Live Expression while enabled. */
extern volatile uint8_t CurtisIO_Override_forward;    /* Forward  out line, 0/1  */
extern volatile uint8_t CurtisIO_Override_backward;   /* Backward out line, 0/1  */
extern volatile uint8_t CurtisIO_Override_pedal;      /* Pedal    out line, 0/1  */
extern volatile float   CurtisIO_Override_mcor_volts; /* MCOR DAC target, 0..VREF */
extern volatile uint8_t CurtisIO_Override_i2c_ok;     /* 1 = last DAC write ACKed */

/* ========================= Closed-loop speed control ===================== *
 * Drive the wheel to a target linear speed (m/s) with a PID loop. The loop
 * reads the speed sensor (CurtisIO_SpeedMps), compares it to the target, and
 * commands the MCOR throttle DAC to close the error; the sign of the target
 * picks the direction lines:
 *   target > 0  -> forward=1, backward=0, pedal=1, MCOR = PID output
 *   target < 0  -> forward=0, backward=1, pedal=1, MCOR = PID output
 *   target = 0  -> forward=0, backward=0, pedal=0, MCOR = 0  (full stop)
 * The pulse sensor only reports speed *magnitude*, so the loop regulates
 * |target| against the measured magnitude and trusts the direction lines to
 * make the wheel actually turn the commanded way.
 *
 * Like the override/self-test, it owns the mode relay: on the rising edge of
 * `enable` it energises Relay_Mode (OutputEnable(1)) and lets it settle before
 * driving; on the falling edge it zeroes every output, DAC=0, and releases the
 * relay back to the input side. Non-blocking — call it every main-loop pass; it
 * paces the PID update off HAL_GetTick. Do not run this together with
 * CurtisIO_OutputOverrideRun()/CurtisIO_OutputTestRun() (they share the outputs).
 * ------------------------------------------------------------------------- */

/**
  * @brief  One step of the closed-loop speed controller. Call every loop pass.
  * @param  enable     : 1 = run the PID loop, 0 = idle (releases outputs on the
  *                      falling edge, mode relay OFF).
  * @param  target_mps : desired wheel speed, m/s. Sign selects direction; 0
  *                      commands a full stop (all lines low, DAC 0).
  */
void CurtisIO_SpeedControlRun(uint8_t enable, float target_mps);

/* PID gains — tune live in a Live Expression. Error is in m/s, output in volts. */
extern volatile float CurtisIO_Speed_Kp;
extern volatile float CurtisIO_Speed_Ki;
extern volatile float CurtisIO_Speed_Kd;

/* Live-Expression telemetry, refreshed each control step. */
extern volatile float   CurtisIO_Speed_target_mps;   /* last commanded target   */
extern volatile float   CurtisIO_Speed_measured_mps; /* measured speed magnitude */
extern volatile float   CurtisIO_Speed_error;        /* |target| - measured     */
extern volatile float   CurtisIO_Speed_mcor_volts;   /* PID -> MCOR DAC, volts  */
extern volatile uint8_t CurtisIO_Speed_forward;      /* Forward  out line, 0/1  */
extern volatile uint8_t CurtisIO_Speed_backward;     /* Backward out line, 0/1  */
extern volatile uint8_t CurtisIO_Speed_pedal;        /* Pedal    out line, 0/1  */
extern volatile uint8_t CurtisIO_Speed_i2c_ok;       /* 1 = last DAC write ACKed */

#ifdef __cplusplus
}
#endif

#endif /* INC_CURTIS_IO_H_ */

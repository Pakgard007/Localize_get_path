/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>   // memcpy() for packing the float current into the CAN frame
#include "flash_store.h"   // persist start/stop servo angles in flash (split-out lib)
#include "curtis_io.h"     // read/drive Curtis 1510 direction/pedal/MCOR/speed-sensor
#include "tim.h"           // htim2 + MX_TIM2_Init() for the speed-sensor input capture
#include "i2c.h"           // hi2c2 + MX_I2C2_Init() for the MCP4725 MCOR-out DAC
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---- CAN protocol (raw classic-CAN frames bridged to ROS 2 on the PC) ----
 * /brake_command (std_msgs/Bool)  : PC -> STM32, ID 0x130, data[0] = 1 ON / 0 OFF
 * /brake_status  (BrakeStatus.msg): STM32 -> PC, ID 0x131, 8-byte heartbeat:
 *     [0..3] float32 current_ma (little-endian)
 *     [4]    uint8  relay_active   (1 = Relay G2R-24 ON, 0 = OFF)
 *     [5]    uint8  status bits: bit0 = watchdog_status (0 = Normal, 1 =
 *            Triggered/fault), bit1 = PC13 E_Stop live status (1 = active)
 *     [6..7] uint16 heartbeat sequence counter (little-endian)
 * /servo_command : PC -> STM32, ID 0x132, 8-byte. Two independent servo angles,
 *            each a little-endian float32 clamped to 0–180°. Stored in flash and
 *            recalled on boot:
 *     [0..3] float32 start_deg : servo position when the brake is OFF/released
 *            (relay open, motor free to start).
 *     [4..7] float32 stop_deg  : servo position when the brake is ON/engaged
 *            (relay closed, motor stopped).
 *            There is no polarity/sign flag any more — start and stop are set
 *            directly and independently.
 * /servo_status  : STM32 -> PC, ID 0x133, 8-byte, sent on the same 20 ms tick as
 *            0x131. The held start/stop angles (flash-recalled at boot, then the
 *            live values after each overwrite), same layout as /servo_command:
 *     [0..3] float32 start_deg (little-endian)
 *     [4..7] float32 stop_deg  (little-endian)
 * /brake_angle   : STM32 -> PC, ID 0x134, 1-byte. Current servo/brake angle the
 *            sequencer is driving to, in whole degrees (0..180). Diagram assigns
 *            this to 0x132, but 0x132/0x133 are already used above for the servo
 *            start/stop config, so /brake_angle lives on the free ID 0x134:
 *     [0] uint8 angle_deg (0..180).
 * ---- Closed-loop speed control (Curtis 1510), scaled int16 in m/s ----------
 * The speed value on the wire is a signed little-endian int16 in units of
 * 0.01 m/s (SPEED_CMD_SCALE): raw = round(m/s * 100), range ±327.67 m/s. The
 * sign carries direction (>0 forward, <0 backward, 0 = stop).
 * /cmd_vel (geometry_msgs/Twist.linear.x) : PC -> STM32, ID 0x120, 2-byte:
 *     [0..1] int16 linear_x : target wheel speed, 0.01 m/s/LSB (little-endian).
 * /speed_enable (std_msgs/Bool)           : PC -> STM32, ID 0x121, data[0] =
 *            1 -> run the PID speed loop (mode relay ON), 0 -> release outputs.
 * /speed_status  : STM32 -> PC, ID 0x122, 8-byte, sent on the same 20 ms tick as
 *            0x131. Measured + target speed and controller flags:
 *     [0..1] int16 measured_speed : current wheel speed, 0.01 m/s/LSB (LE); sign
 *            follows the commanded direction (the pulse sensor is unsigned).
 *     [2..3] int16 target_speed   : commanded speed, 0.01 m/s/LSB (LE).
 *     [4]    uint8 status_flags    : see SPEED_FLAG_* (bit0 enabled, bit1 fwd_cmd,
 *            bit2 rev_cmd, bit3 pedal, bit4 sensor_valid, bit5 timeout, bit6
 *            PC13 E_Stop, bit7 fault)
 *     [5]    uint8 fault_code      : 0 = none (reserved for future fault codes).
 *     [6..7] uint16 sequence counter (little-endian).
 * /speed_diagnostics : STM32 -> PC, ID 0x123, 8-byte, same 20 ms tick. Raw
 *            Curtis I/O snapshot for debugging:
 *     [0]    uint8 input_flags  : bit0 FWD in, bit1 REV in, bit2 Pedal in.
 *     [1]    uint8 output_flags : bit0 FWD out, bit1 REV out, bit2 Pedal out,
 *            bit3 mode_relay.
 *     [2..3] uint16 MCOR input  mV (little-endian) — throttle wiper read.
 *     [4..5] uint16 MCOR output mV (little-endian) — last DAC value driven.
 *     [6..7] uint16 speed_sensor_hz (little-endian) — pulse frequency. */
#define CAN_ID_CMD_VEL        0x120u   /* RX: target speed (int16, 0.01 m/s/LSB) */
#define CAN_ID_SPEED_ENABLE   0x121u   /* RX: enable PID speed control (bool)    */
#define CAN_ID_SPEED_STATUS   0x122u   /* TX: /speed_status, 8 bytes (see below)  */
#define CAN_ID_SPEED_DIAG     0x123u   /* TX: /speed_diagnostics, 8 bytes         */
#define CAN_ID_BRAKE_CMD      0x130u
#define CAN_ID_BRAKE_STATUS   0x131u
#define CAN_ID_SERVO_CMD      0x132u
#define CAN_ID_SERVO_STATUS   0x133u
#define CAN_ID_BRAKE_ANGLE    0x134u   /* TX: /brake_angle, 1 byte servo angle    */

/* Speed value wire scaling: signed int16, 0.01 m/s per LSB (round-trip m/s). */
#define SPEED_CMD_SCALE       100.0f

/* /speed_status status_flags (byte 4) bit positions (see 0x122 layout below). */
#define SPEED_FLAG_ENABLED    0x01u   /* bit0: speed controller enabled          */
#define SPEED_FLAG_FWD_CMD    0x02u   /* bit1: forward output commanded          */
#define SPEED_FLAG_REV_CMD    0x04u   /* bit2: reverse output commanded          */
#define SPEED_FLAG_PEDAL      0x08u   /* bit3: pedal output active               */
#define SPEED_FLAG_SENSOR_OK  0x10u   /* bit4: speed sensor reading valid        */
#define SPEED_FLAG_TIMEOUT    0x20u   /* bit5: /speed_cmd timeout active         */
#define SPEED_FLAG_ESTOP      0x40u   /* bit6: PC13 E-Stop active                */
#define SPEED_FLAG_FAULT      0x80u   /* bit7: speed fault active                */

/* /speed_cmd (0x120) watchdog: if no target frame arrives within this window the
 * SPEED_FLAG_TIMEOUT bit is set so the Jetson bridge can react (the diagram's
 * "cmd_vel timeout -> brake" rule). */
#define SPEED_CMD_TIMEOUT_MS  500u

/* ---- INA240A2D current sensor (gain 50 V/V) with a 2 mOhm shunt ----------
 * Unidirectional wiring (REF tied to GND) so 0 A -> ~0 V and only the
 * positive direction is measured.
 *   Vout      = I * Rshunt * Gain  ->  0.002 * 50 = 0.1 V/A
 *   I [A]     = (Vadc - offset) / 0.1
 * Change CURRENT_ADC_INDEX if the INA240 output is wired to a different
 * ADC1 rank (adc_buffer[0]=PA0/IN1, [1]=PA1, [2]=PA2, [3]=PA3).            */
#define ADC_VREF              3.3f
#define ADC_FULL_SCALE        4095.0f
#define INA240_GAIN           50.0f
#define SHUNT_OHMS            0.002f
#define CURRENT_V_PER_A       (INA240_GAIN * SHUNT_OHMS)   /* 0.1 V/A */
#define CURRENT_OFFSET_V      0.0f                         /* unidirectional */
#define CURRENT_ADC_INDEX     0u

/* Curtis 1510 MCOR throttle wiper: wired to PA1 = ADC1_IN2 = regular rank 2, so
 * it lands in adc_buffer[1] (see the ADC1 rank order above). */
#define MCOR_ADC_INDEX        1u

/* Open-load / motor-fault check: Relay commanded ON but ~no current flowing.
 * Temporarily disabled (set to 1 to re-enable once a real load is connected). */
#define ENABLE_OPENLOAD_CHECK        0
#define CURRENT_FAULT_THRESHOLD_MA   50.0f

/* Overcurrent trip: cut the relay if the current stays >= threshold continuously
 * for OVERCURRENT_TRIP_MS. The delay rejects the brief inrush/noise spike the
 * servo motor produces when it starts moving (engage or release), while still
 * catching a real sustained fault (stall / short). */
#define OVERCURRENT_THRESHOLD_A      14.0f
#define OVERCURRENT_TRIP_MS          150u

/* Moving-average window over the raw ADC current samples. Smooths single-sample
 * spikes (relay switching noise) that were tripping the overcurrent latch. */
#define CURRENT_AVG_WINDOW           16u

/* Heartbeat: TIM6 elapses every 10 ms. The status frames alternate between a
 * brake group and a speed group on each flag (see the main-loop TX block), so a
 * flag every 10 ms (1 tick) gives each group a 20 ms transmit rate. */
#define HEARTBEAT_TICKS       1u

/* On a normal release (relay commanded OFF) the servo is driven back to the
 * start position and we keep the relay energised for this long so it can
 * physically get there before power is cut. Faults (E-Stop/overcurrent) skip
 * this and cut at once. */
#define SERVO_SETTLE_MS       3000u

/* Persisted start/stop servo angles: the flash layout, magic and load/save logic
 * live in the flash_store.[ch] library. What stays here is the app-level policy
 * for *when* to commit.
 * NOTE: the G474 has no read-while-write, so a page erase (~22 ms) stalls the
 * whole CPU — interrupts included — for its duration. To avoid stalling during a
 * burst of /servo_command frames (which would make CAN look unresponsive), the
 * write is debounced: we only commit once the angles have been stable for
 * FLASH_WRITE_QUIET_MS. The boot defaults (used until a /servo_command is saved)
 * are app-level tuning knobs and are passed into FlashStore_LoadAngles(). */
#define START_ANGLE_DEFAULT   180.0f  /* brake OFF/released */
#define STOP_ANGLE_DEFAULT    110.0f  /* brake ON/engaged  */
#define FLASH_WRITE_QUIET_MS  800u

/* On a verified flash write, blink LED2 rapidly for a moment as a visual ACK.
 * Driven from the 10 ms TIM6 tick: toggle every LED2_BLINK_PERIOD ticks for
 * LED2_BLINK_TICKS ticks total (100 ticks = ~1 s, toggling every 50 ms ≈ 10 Hz). */
#define LED2_BLINK_TICKS      100u
#define LED2_BLINK_PERIOD     5u

/* Local push-button on PC1 (USER_SW) that toggles the relay. Wired active-high:
 * press connects the pin to 3.3V, internal pull-down holds it low when released. */
#define BTN_TOGGLE_Pin        GPIO_PIN_1
#define BTN_TOGGLE_GPIO_Port  GPIOC

/* E-Stop on PD2: active-low with pull-up (idle HIGH, triggered LOW -> falling). */
#define ESTOP_Pin             GPIO_PIN_2
#define ESTOP_GPIO_Port       GPIOD

/* Second E-Stop on PC13 (labelled E_Stop in the .ioc; pin defines E_Stop_Pin /
 * E_Stop_GPIO_Port come from main.h). This one is polled and, while active,
 * acted on locally (non-latching): the firmware forces the brake fully engaged
 * to STOP_ANGLE_DEFAULT, holds the relay closed, and zeroes the speed target.
 * When PC13 releases, the normal relay_cmd / speed command flow resumes. It is
 * also still reported in the status frames (0x131 bit / 0x122 flag).
 * Real vehicle wiring is active-low: idle HIGH via pull-up, pressed/open safety
 * loop pulls LOW. */
#define ESTOP_PC13_ACTIVE_LOW 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t relay_cmd = 0;       // 0 = OFF, 1 = ON. Set by /brake_command over CAN (or Live Expression)
volatile float   start_angle_deg = START_ANGLE_DEFAULT; // servo angle when brake OFF/released (0–180°), persisted in flash, set by /servo_command
volatile float   stop_angle_deg  = STOP_ANGLE_DEFAULT;  // servo angle when brake ON/engaged  (0–180°), persisted in flash, set by /servo_command
volatile uint8_t  flash_save_req = 0;  // set by RX ISR when start/stop angle changes; flash write runs in main loop
volatile uint32_t flash_req_tick = 0;  // HAL tick of the last start/stop angle change (debounce reference)
volatile uint8_t flash_write_ok = 0;  // 1 = last flash write read back OK (visible in Live Expression)
volatile uint16_t led2_blink_ticks = 0; // >0 = LED2 rapid-blink countdown (10 ms ticks), set on a verified write
volatile uint16_t adc_buffer[4];     // สร้าง Buffer รอรับค่าจาก ADC DMA
uint16_t led_counter = 0;

/* Relay/servo release sequencer state (driven in the main loop). */
typedef enum {
	BRAKE_OFF = 0,    // relay open, servo parked at start_angle_deg
	BRAKE_ON,         // relay closed, servo driven to stop_angle_deg
	BRAKE_RELEASING   // servo driving back to start_angle_deg, relay still on until settle elapses
} brake_state_t;

/* --- Brake status / safety state shared with the ISRs --- */
volatile uint8_t  watchdog_status = 0;   // 0 = Normal, 1 = Triggered (E-Stop or open-load fault). Latched.
volatile uint8_t  estop_pc13     = 0;    // live PC13 E_Stop status (1 = active), reported in 0x131/0x122
volatile uint8_t  can_tx_flag = 0;       // set by TIM6 every HEARTBEAT_TICKS, consumed in main loop
float             current_ma = 0.0f;     // latest INA240 current reading, milliamps
uint16_t          heartbeat_seq = 0;     // rolling counter so the PC can detect dropped frames

FDCAN_TxHeaderTypeDef CanTxHeader;        // configured once in CAN_App_Init() — used for 0x131
FDCAN_TxHeaderTypeDef CanServoTxHeader;   // configured once in CAN_App_Init() — used for 0x133
FDCAN_TxHeaderTypeDef CanSpeedTxHeader;   // configured once in CAN_App_Init() — used for 0x122
FDCAN_TxHeaderTypeDef CanSpeedDiagTxHeader; // configured once in CAN_App_Init() — used for 0x123
FDCAN_TxHeaderTypeDef CanBrakeAngleTxHeader; // configured once in CAN_App_Init() — used for 0x134

/* --- Speed-status telemetry shared with the ISRs / status senders --- */
uint16_t          speed_seq       = 0;   // /speed_status sequence counter (0x122 [6..7])
uint8_t           speed_fault_code = 0;  // 0 = none; reserved for future speed faults
volatile uint32_t last_cmd_vel_tick = 0; // HAL tick of the last /speed_cmd RX (0 = none yet)
volatile uint8_t  cmd_vel_seen    = 0;   // 1 once the first /speed_cmd has arrived
volatile uint8_t  brake_servo_deg = STOP_ANGLE_DEFAULT; // current servo target, whole deg (0x134)

/* --- Curtis 1510 input readings, refreshed each main-loop pass (Live Expression) --- */
volatile uint8_t curtis_forward  = 0;   // Forward_IN_to_MCU  (PC5) raw level
volatile uint8_t curtis_backward = 0;   // Backward_IN_to_MCU (PC4) raw level
volatile uint8_t curtis_pedal    = 0;   // Pedal_IN_to_MCU    (PB0) raw level
volatile float   mcor_volts      = 0.0f;// MCOR throttle wiper (PA1/IN2), volts
volatile float   speed_sensor_hz = 0.0f;// Speed_Sensor_to_MCU (PA15/TIM2 CH1), pulse Hz

/* --- Output self-test (bench). Flip to 1 in a Live Expression to energise the
 * mode relay and start cycling the Forward/Backward/Pedal out lines + sweeping
 * the MCOR DAC; back to 0 releases every output and hands the Curtis back to
 * the input/pass-through side. Watch the CurtisIO_OutTest_* variables move. --- */
volatile uint8_t output_test_enable = 0;
#define OUTPUT_TEST_PERIOD_MS   100u   // ms between test steps (pattern advance)

/* --- Manual output override (bench). Set CurtisIO_Override_forward/backward/
 * pedal (0/1) and CurtisIO_Override_mcor_volts (0..VREF) in a Live Expression,
 * then flip this to 1 to drive the Curtis outputs to exactly those values. Takes
 * precedence over the auto self-test above (only one drives the outputs). Back to
 * 0 releases every output and hands the Curtis back to the input side. --- */
volatile uint8_t output_override_enable = 0;

/* --- Closed-loop speed control (bench/ROS). Set curtis_speed_target_mps to the
 * desired wheel speed in m/s (sign = direction, 0 = stop), then flip
 * speed_control_enable to 1: the PID loop energises the mode relay and drives
 * the Curtis Forward/Backward/Pedal lines + MCOR throttle to hold that speed.
 * Takes precedence over the manual override and auto self-test (only one drives
 * the shared outputs). Back to 0 releases every output (mode relay OFF). Tune
 * with CurtisIO_Speed_Kp/Ki/Kd; watch CurtisIO_Speed_* for telemetry. --- */
volatile uint8_t speed_control_enable    = 0;
volatile float   curtis_speed_target_mps = 0.0f;   // desired wheel speed, m/s (signed)
volatile float   speed_sensor_mps        = 0.0f;   // measured speed magnitude, m/s (Live Expression)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void  CAN_App_Init(void);          // FDCAN filter + start + RX notification
static void  BrakeStatus_Send(void);      // pack and transmit the /brake_status heartbeat frame
static void  ServoStatus_Send(void);      // transmit the held start/stop angles (0x133)
static void  BrakeAngle_Send(void);       // transmit the current servo/brake angle (0x134)
static void  SpeedStatus_Send(void);      // transmit /speed_status (0x122, 8-byte)
static void  SpeedDiag_Send(void);        // transmit /speed_diagnostics (0x123, 8-byte)
/* Persisted start/stop servo angles now live in flash_store.[ch]:
 *   FlashStore_LoadAngles() / FlashStore_SaveAngles() */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t deg_to_pulse(float deg) {
    if (deg < 0.0f)   deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    return (uint32_t)(1000.0f + deg * (1000.0f / 180.0f));
}

/* Wire encoding for the speed frames: m/s -> signed int16 in 0.01 m/s units,
 * rounded and clamped to the int16 range so a huge value cannot wrap. */
static int16_t mps_to_raw(float mps) {
    float scaled = mps * SPEED_CMD_SCALE;
    scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;   /* round to nearest */
    if (scaled >  32767.0f) scaled =  32767.0f;
    if (scaled < -32768.0f) scaled = -32768.0f;
    return (int16_t) scaled;
}

/* Inverse of mps_to_raw: signed int16 (0.01 m/s/LSB) -> m/s. */
static float raw_to_mps(int16_t raw) {
    return (float) raw / SPEED_CMD_SCALE;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_I2C2_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	/* E-Stop (PD2): CubeMX generates this EXTI pin as GPIO_NOPULL, which leaves
	 * the active-low line floating and fires spurious falling edges -> false
	 * watchdog_status=1. Re-init with internal pull-up (USER CODE = regen-safe),
	 * clear stale pending edges, then sample the real level so an already-pressed
	 * e-stop at boot is latched immediately. */
	GPIO_InitTypeDef estop_init = {0};
	__HAL_RCC_GPIOD_CLK_ENABLE();
	estop_init.Pin = ESTOP_Pin;
	estop_init.Mode = GPIO_MODE_IT_FALLING;
	estop_init.Pull = GPIO_PULLUP;
	estop_init.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ESTOP_GPIO_Port, &estop_init);
	__HAL_GPIO_EXTI_CLEAR_IT(ESTOP_Pin);
	HAL_NVIC_ClearPendingIRQ(EXTI2_IRQn);
	watchdog_status =
			(HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_RESET) ? 1u : 0u;

	/* PC13 E_Stop is the live, non-latching e-stop input. Re-init it with an
	 * internal pull-up so active-low real e-stop wiring has a defined idle level. */
	GPIO_InitTypeDef pc13_estop_init = {0};
	__HAL_RCC_GPIOC_CLK_ENABLE();
	pc13_estop_init.Pin = E_Stop_Pin;
	pc13_estop_init.Mode = GPIO_MODE_INPUT;
	pc13_estop_init.Pull = GPIO_PULLUP;
	pc13_estop_init.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(E_Stop_GPIO_Port, &pc13_estop_init);

	/* Recall the start/stop angles saved by the last /servo_command (defaults if
	 * the flash page was never written). */
	float loaded_start, loaded_stop;
	FlashStore_LoadAngles(&loaded_start, &loaded_stop,
			START_ANGLE_DEFAULT, STOP_ANGLE_DEFAULT);
	start_angle_deg = loaded_start;
	stop_angle_deg  = loaded_stop;

	CAN_App_Init();                 // ตั้งค่า Filter + Start FDCAN + เปิด RX Interrupt
	HAL_TIM_Base_Start_IT(&htim6);  // เริ่ม Timer ของ WDI
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // เริ่ม PWM ของ Servo
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer, 4);

	/* Speed sensor (PA15 -> TIM2 CH1): start input capture with interrupt. CubeMX
	 * generated TIM2 without its NVIC line enabled, so enable it here (USER CODE =
	 * regen-safe). The TIM2_IRQHandler lives in stm32g4xx_it.c (also USER CODE). */
	HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/* Poll the PC13 E_Stop line first so the brake sequencer and speed loop
		 * below act on the current level (no one-cycle lag). Active-low real
		 * e-stop wiring is the default. While active the firmware forces the
		 * brake engaged (see the sequencer) and commands speed 0; it clears the
		 * moment the line releases. Also reported in 0x131/0x122. */
		GPIO_PinState estop_lvl = HAL_GPIO_ReadPin(E_Stop_GPIO_Port, E_Stop_Pin);
#if ESTOP_PC13_ACTIVE_LOW
		estop_pc13 = (estop_lvl == GPIO_PIN_RESET) ? 1u : 0u;
#else
		estop_pc13 = (estop_lvl == GPIO_PIN_SET) ? 1u : 0u;
#endif
		if (estop_pc13) {
			curtis_speed_target_mps = 0.0f;   /* command speed 0 while E-Stop active */
		}

		/* Relay/servo sequencer:
		 *   OFF  -> ON         : commanded ON (no fault) -> relay closes, servo to brake_angle
		 *   ON                 : servo follows brake_angle live (re-sent /servo_command)
		 *   ON   -> RELEASING  : commanded OFF -> servo to 0°, relay stays on
		 *   RELEASING -> OFF   : after SERVO_SETTLE_MS the relay opens
		 * A fault (watchdog_status=1) cuts the relay immediately from any state. */
		static brake_state_t brake_state = BRAKE_OFF;
		static float    servo_target  = START_ANGLE_DEFAULT;
		static uint32_t release_tick  = 0;

		/* Engage/release endpoints, set independently via /servo_command:
		 *   brake ON  (engaged) -> stop_deg
		 *   brake OFF (released)-> start_deg */
		float engaged_deg  = stop_angle_deg;
		float released_deg = start_angle_deg;

		if (watchdog_status == 1) {
			/* E-Stop / latched fault: open relay now, no graceful return. */
			HAL_GPIO_WritePin(RELAY_Brake_GPIO_Port, RELAY_Brake_Pin, GPIO_PIN_RESET);
			servo_target = released_deg;
			brake_state  = BRAKE_OFF;
		} else if (estop_pc13) {
			/* PC13 E-Stop active (non-latching): force the brake fully engaged to
			 * STOP_ANGLE_DEFAULT and hold the relay closed. Speed is zeroed above.
			 * Enter BRAKE_ON so that when PC13 releases the sequencer returns to
			 * normal — staying engaged if relay_cmd==1, or releasing gracefully to
			 * start_angle_deg if relay_cmd==0. */
			HAL_GPIO_WritePin(RELAY_Brake_GPIO_Port, RELAY_Brake_Pin, GPIO_PIN_SET);
			servo_target = STOP_ANGLE_DEFAULT;
			brake_state  = BRAKE_ON;
		} else {
			switch (brake_state) {
			case BRAKE_OFF:
				HAL_GPIO_WritePin(RELAY_Brake_GPIO_Port, RELAY_Brake_Pin, GPIO_PIN_RESET);
				servo_target = released_deg;
				if (relay_cmd == 1) {
					HAL_GPIO_WritePin(RELAY_Brake_GPIO_Port, RELAY_Brake_Pin, GPIO_PIN_SET);
					servo_target = engaged_deg;
					brake_state  = BRAKE_ON;
				}
				break;

			case BRAKE_ON:
				HAL_GPIO_WritePin(RELAY_Brake_GPIO_Port, RELAY_Brake_Pin, GPIO_PIN_SET);
				servo_target = engaged_deg;   /* follow live angle updates */
				if (relay_cmd == 0) {
					servo_target = released_deg; /* drive to release position before powering off */
					release_tick = HAL_GetTick();
					brake_state  = BRAKE_RELEASING;
				}
				break;

			case BRAKE_RELEASING:
				HAL_GPIO_WritePin(RELAY_Brake_GPIO_Port, RELAY_Brake_Pin, GPIO_PIN_SET);
				servo_target = released_deg;
				if (relay_cmd == 1) {              /* re-engaged mid-release */
					servo_target = engaged_deg;
					brake_state  = BRAKE_ON;
				} else if ((HAL_GetTick() - release_tick) >= SERVO_SETTLE_MS) {
					HAL_GPIO_WritePin(RELAY_Brake_GPIO_Port, RELAY_Brake_Pin, GPIO_PIN_RESET);
					brake_state = BRAKE_OFF;
				}
				break;
			}
		}

		/* Moving average over the last CURRENT_AVG_WINDOW raw ADC samples to reject
		 * single-sample spikes before converting to current. Ring buffer keeps a
		 * running sum so each iteration is O(1). */
		static uint16_t adc_window[CURRENT_AVG_WINDOW] = {0};
		static uint32_t adc_sum = 0;
		static uint8_t  adc_idx = 0;
		uint16_t adc_raw = adc_buffer[CURRENT_ADC_INDEX];
		adc_sum -= adc_window[adc_idx];
		adc_window[adc_idx] = adc_raw;
		adc_sum += adc_raw;
		adc_idx = (uint8_t) ((adc_idx + 1u) % CURRENT_AVG_WINDOW);
		float adc_avg = (float) adc_sum / (float) CURRENT_AVG_WINDOW;

		/* Convert the averaged INA240 ADC reading into milliamps (unidirectional). */
		float vout = (adc_avg / ADC_FULL_SCALE) * ADC_VREF;
		float amps = (vout - CURRENT_OFFSET_V) / CURRENT_V_PER_A;
		if (amps < 0.0f) {
			amps = 0.0f;
		}
		current_ma = amps * 1000.0f;

		/* Overcurrent protection with a persistence filter: only trip when the
		 * current has been over the threshold continuously for OVERCURRENT_TRIP_MS,
		 * so the servo's brief start-up inrush/noise spike does not cut the relay. */
		static uint32_t oc_over_since = 0;   /* tick the current first crossed over; 0 = below */
		if (amps >= OVERCURRENT_THRESHOLD_A) {
			uint32_t now = HAL_GetTick();
			if (oc_over_since == 0) {
				oc_over_since = (now != 0u) ? now : 1u;  /* 0 is the "below" sentinel */
			}
			if ((now - oc_over_since) >= OVERCURRENT_TRIP_MS) {
				HAL_GPIO_WritePin(RELAY_Brake_GPIO_Port, RELAY_Brake_Pin, GPIO_PIN_RESET);
				relay_cmd    = 0;
				servo_target = released_deg;
				brake_state  = BRAKE_OFF;
			}
		} else {
			oc_over_since = 0;   /* dropped back below threshold -> restart the timer */
		}

		/* Drive the servo PWM to the sequencer's current target. */
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, deg_to_pulse(servo_target));

		/* Publish the current servo angle on /brake_angle (0x134): whole degrees,
		 * clamped to 0..180 so it fits one byte. */
		float ang = servo_target;
		if (ang < 0.0f)   ang = 0.0f;
		if (ang > 180.0f) ang = 180.0f;
		brake_servo_deg = (uint8_t) (ang + 0.5f);

		/* Persist freshly received start/stop angles outside the RX ISR. Erasing
		 * bank 2 is RWW-safe, so this does not stall the heartbeat below. */
		/* Debounced flash persist: commit only after the angles have been stable for
		 * FLASH_WRITE_QUIET_MS, so a burst of /servo_command frames does not trigger
		 * back-to-back ~22 ms erase stalls (which would drop/delay CAN frames). */
		if (flash_save_req && (HAL_GetTick() - flash_req_tick) >= FLASH_WRITE_QUIET_MS) {
			flash_save_req = 0;
			flash_write_ok = FlashStore_SaveAngles(start_angle_deg, stop_angle_deg);
			if (flash_write_ok) {
				led2_blink_ticks = LED2_BLINK_TICKS;  // rapid-blink LED2 as a write ACK
			}
		}

		/* Refresh the Curtis 1510 input readings (Live Expression / future use):
		 *   - Forward/Backward/Pedal digital lines (raw pin levels)
		 *   - MCOR throttle wiper voltage (ADC1_IN2 via the DMA buffer)
		 *   - speed-sensor pulse frequency (TIM2 CH1 input capture) */
		/* (PC13 E_Stop is polled at the top of the loop and acted on there.) */
		CurtisDigitalInputs_t curtis_in;
		CurtisIO_ReadDigital(&curtis_in);
		curtis_forward  = curtis_in.forward;
		curtis_backward = curtis_in.backward;
		curtis_pedal    = curtis_in.pedal;
		mcor_volts      = CurtisIO_McorVolts(adc_buffer[MCOR_ADC_INDEX]);
		speed_sensor_hz = CurtisIO_SpeedHz();
		speed_sensor_mps = CurtisIO_SpeedMps();

		/* Curtis output drivers (Live Expression, bench). Three mutually exclusive
		 * modes share the mode relay + output lines, so only one may drive at a
		 * time. Precedence: speed control > manual override > auto self-test.
		 *   - speed_control_enable   : PID loop holds curtis_speed_target_mps (m/s)
		 *   - output_override_enable : drive outputs to the CurtisIO_Override_* values
		 *   - output_test_enable     : rotating auto self-test pattern / MCOR sweep
		 * Each is non-blocking and releases its outputs on the falling edge; gate
		 * the lower-priority ones off so a released mode does not fight the relay.
		 * A PC13 E-Stop also cuts the manual override and self-test drivers so they
		 * cannot keep driving the motor past the E-Stop (the override/self-test
		 * paths ignore the speed target, so zeroing it alone would not stop them).
		 * The PID path stays live but with the target forced to 0 above, so if it
		 * was enabled it actively holds speed 0. */
		uint8_t sc_on = speed_control_enable;
		uint8_t ov_on = output_override_enable && !sc_on && !estop_pc13;
		CurtisIO_SpeedControlRun(sc_on, curtis_speed_target_mps);
		CurtisIO_OutputOverrideRun(ov_on);
//		CurtisIO_OutputTestRun(output_test_enable && !ov_on && !sc_on && !estop_pc13, OUTPUT_TEST_PERIOD_MS);

		/* Status transmit, split across two 10 ms ticks so we never queue more
		 * than 3 frames at once — the STM32G4 FDCAN Tx FIFO is only 3 deep, and
		 * queuing all 5 frames back-to-back overflowed it and silently dropped
		 * the last two (0x122 /speed_status + 0x123 /speed_diagnostics).
		 *   tick A: brake group  (0x133 servo, 0x131 heartbeat, 0x134 angle) = 3
		 *   tick B: speed group  (0x122 status, 0x123 diagnostics)           = 2
		 * Each group therefore still transmits every 20 ms. 0x133 goes before
		 * 0x131 so the bridge has the current angle before it publishes
		 * /brake_status on receiving 0x131. */
		static uint8_t tx_group = 0;
		if (can_tx_flag) {
			can_tx_flag = 0;
			if (tx_group == 0) {
				ServoStatus_Send();   /* 0x133 */
				BrakeStatus_Send();   /* 0x131 */
				BrakeAngle_Send();    /* 0x134 */
			} else {
				SpeedStatus_Send();   /* 0x122 */
				SpeedDiag_Send();     /* 0x123 */
			}
			tx_group ^= 1u;
		}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV8;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief Configure the FDCAN acceptance filter, start the peripheral and
  *        enable the RX FIFO0 "new message" interrupt. Also prepares the
  *        static TX header used for every /brake_status frame.
  */
static void CAN_App_Init(void)
{
	/* Bit timing / bitrate now come from the .ioc (NominalPrescaler = 40 ->
	 * 250 kbps @ 170 MHz, sample point 82.4 %), so MX_FDCAN1_Init() already set
	 * them. Two settings the .ioc does not carry are applied here before the RX
	 * filters are configured, then HAL_FDCAN_Init() is re-run to commit them:
	 *   - AutoRetransmission: ENABLE (CubeMX default is DISABLE)
	 *   - StdFiltersNbr:      4, so the acceptance filters below can allocate.
	 * Must still match the SocketCAN bitrate on the Jetson (bitrate 250000). */
	hfdcan1.Init.AutoRetransmission = ENABLE;
	/* slot 0: /brake_command, 1: /servo_command, 2: /cmd_vel, 3: /speed_enable */
	hfdcan1.Init.StdFiltersNbr = 4;
	if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}

	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID2 = 0x7FF; /* full mask -> exact ID match */

	/* Accept /brake_command (0x130) */
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterID1 = CAN_ID_BRAKE_CMD;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* Accept /servo_command (0x132) */
	sFilterConfig.FilterIndex = 1;
	sFilterConfig.FilterID1 = CAN_ID_SERVO_CMD;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* Accept /cmd_vel (0x120): target wheel speed for the PID loop */
	sFilterConfig.FilterIndex = 2;
	sFilterConfig.FilterID1 = CAN_ID_CMD_VEL;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* Accept /speed_enable (0x121): turn the PID speed loop on/off */
	sFilterConfig.FilterIndex = 3;
	sFilterConfig.FilterID1 = CAN_ID_SPEED_ENABLE;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
			FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
		Error_Handler();
	}

	/* TX header for /brake_status (0x131, classic CAN, 8 bytes). */
	CanTxHeader.Identifier = CAN_ID_BRAKE_STATUS;
	CanTxHeader.IdType = FDCAN_STANDARD_ID;
	CanTxHeader.TxFrameType = FDCAN_DATA_FRAME;
	CanTxHeader.DataLength = FDCAN_DLC_BYTES_8;
	CanTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	CanTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	CanTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	CanTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	CanTxHeader.MessageMarker = 0;

	/* Servo-status frame (0x133): 8-byte, two float32 (start_deg, stop_deg). */
	CanServoTxHeader = CanTxHeader;
	CanServoTxHeader.Identifier = CAN_ID_SERVO_STATUS;
	CanServoTxHeader.DataLength = FDCAN_DLC_BYTES_8;

	/* Brake-angle frame (0x134): 1-byte, current servo angle in whole degrees. */
	CanBrakeAngleTxHeader = CanTxHeader;
	CanBrakeAngleTxHeader.Identifier = CAN_ID_BRAKE_ANGLE;
	CanBrakeAngleTxHeader.DataLength = FDCAN_DLC_BYTES_1;

	/* Speed-status frame (0x122): 8-byte (measured/target int16 + flags + seq). */
	CanSpeedTxHeader = CanTxHeader;
	CanSpeedTxHeader.Identifier = CAN_ID_SPEED_STATUS;
	CanSpeedTxHeader.DataLength = FDCAN_DLC_BYTES_8;

	/* Speed-diagnostics frame (0x123): 8-byte raw Curtis I/O snapshot. */
	CanSpeedDiagTxHeader = CanTxHeader;
	CanSpeedDiagTxHeader.Identifier = CAN_ID_SPEED_DIAG;
	CanSpeedDiagTxHeader.DataLength = FDCAN_DLC_BYTES_8;

	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan1,
			FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
		Error_Handler();
	}
}

/**
  * @brief Pack the brake status into an 8-byte frame and queue it for TX.
  *        Also runs the "Relay ON but no current" open-load validation.
  */
static void BrakeStatus_Send(void)
{
	uint8_t relay_active =
			(HAL_GPIO_ReadPin(RELAY_Brake_GPIO_Port, RELAY_Brake_Pin) == GPIO_PIN_SET) ? 1u : 0u;

	/* Current Validation: Relay is ON but almost no current is flowing ->
	 * broken motor wire or failed motor. Latch the fault so the PC sees it.
	 * Disabled while bench-testing without a load (see ENABLE_OPENLOAD_CHECK). */
#if ENABLE_OPENLOAD_CHECK
	if (relay_active && (current_ma < CURRENT_FAULT_THRESHOLD_MA)) {
		watchdog_status = 1;
	}
#endif

	uint8_t TxData[8];
	memcpy(&TxData[0], &current_ma, sizeof(float)); /* [0..3] float32, little-endian */
	TxData[4] = relay_active;                        /* [4]    relay_active */
	/* [5] status bits: bit0 = watchdog_status (E-Stop/fault latch, as before),
	 *                  bit1 = PC13 E_Stop live status. */
	TxData[5] = (uint8_t) ((watchdog_status & 0x01u) | ((estop_pc13 & 0x01u) << 1));
	TxData[6] = (uint8_t) (heartbeat_seq & 0xFFu);   /* [6..7] sequence counter */
	TxData[7] = (uint8_t) (heartbeat_seq >> 8);
	heartbeat_seq++;

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CanTxHeader, TxData);
}

/**
  * @brief Report the held start/stop angles (0x133): flash values at boot, then
  *        the live /servo_command values after each overwrite.
  */
static void ServoStatus_Send(void)
{
	float s = start_angle_deg;   /* volatile -> local snapshot */
	float t = stop_angle_deg;
	uint8_t TxData[8];
	memcpy(&TxData[0], &s, sizeof(float)); /* [0..3] float32 start_deg, little-endian */
	memcpy(&TxData[4], &t, sizeof(float)); /* [4..7] float32 stop_deg,  little-endian */

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CanServoTxHeader, TxData);
}

/**
  * @brief Report the current servo/brake angle on /brake_angle (0x134): one byte,
  *        whole degrees (0..180), tracking whatever the sequencer is driving to.
  */
static void BrakeAngle_Send(void)
{
	uint8_t TxData[1];
	TxData[0] = brake_servo_deg;   /* [0] uint8 angle_deg (0..180) */
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CanBrakeAngleTxHeader, TxData);
}

/**
  * @brief Report /speed_status (0x122, 8-byte): measured + target speed (signed
  *        int16, 0.01 m/s/LSB), a status_flags byte, a fault_code, and a rolling
  *        sequence counter. The pulse sensor is unsigned, so the sign of both
  *        speeds is taken from the commanded direction (curtis_speed_target_mps).
  */
static void SpeedStatus_Send(void)
{
	float mag = speed_sensor_mps;   /* measured magnitude, m/s (>= 0) */
	/* Re-apply the commanded sign so the PC sees a signed cmd_vel-style speed. */
	float signed_mps = (curtis_speed_target_mps < 0.0f) ? -mag : mag;
	int16_t measured_raw = mps_to_raw(signed_mps);
	int16_t target_raw   = mps_to_raw(curtis_speed_target_mps);

	/* status_flags: pull the live output-line state from the speed controller
	 * telemetry (only valid while the loop drives), plus sensor/timeout/fault. */
	uint8_t timeout_active =
			(cmd_vel_seen && (HAL_GetTick() - last_cmd_vel_tick) >= SPEED_CMD_TIMEOUT_MS)
					? 1u : 0u;
	uint8_t flags = 0u;
	if (speed_control_enable)        flags |= SPEED_FLAG_ENABLED;
	if (CurtisIO_Speed_forward)      flags |= SPEED_FLAG_FWD_CMD;
	if (CurtisIO_Speed_backward)     flags |= SPEED_FLAG_REV_CMD;
	if (CurtisIO_Speed_pedal)        flags |= SPEED_FLAG_PEDAL;
	if (speed_sensor_hz > 0.0f)      flags |= SPEED_FLAG_SENSOR_OK;
	if (timeout_active)              flags |= SPEED_FLAG_TIMEOUT;
	if (estop_pc13)                  flags |= SPEED_FLAG_ESTOP;
	if (speed_fault_code != 0u)      flags |= SPEED_FLAG_FAULT;

	uint8_t TxData[8];
	TxData[0] = (uint8_t) (measured_raw & 0xFFu);        /* [0..1] measured, LE */
	TxData[1] = (uint8_t) ((measured_raw >> 8) & 0xFFu);
	TxData[2] = (uint8_t) (target_raw & 0xFFu);          /* [2..3] target, LE   */
	TxData[3] = (uint8_t) ((target_raw >> 8) & 0xFFu);
	TxData[4] = flags;                                   /* [4]    status_flags */
	TxData[5] = speed_fault_code;                        /* [5]    fault_code   */
	TxData[6] = (uint8_t) (speed_seq & 0xFFu);           /* [6..7] sequence, LE */
	TxData[7] = (uint8_t) ((speed_seq >> 8) & 0xFFu);
	speed_seq++;

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CanSpeedTxHeader, TxData);
}

/**
  * @brief Report /speed_diagnostics (0x123, 8-byte): raw Curtis I/O snapshot —
  *        the digital input lines, the driven output lines + mode relay, the MCOR
  *        throttle voltages (in from the wiper, out from the DAC) and the speed
  *        sensor pulse frequency. For bench/telemetry debugging.
  */
static void SpeedDiag_Send(void)
{
	uint8_t input_flags = 0u;
	if (curtis_forward)  input_flags |= 0x01u;   /* bit0 FWD input   */
	if (curtis_backward) input_flags |= 0x02u;   /* bit1 REV input   */
	if (curtis_pedal)    input_flags |= 0x04u;   /* bit2 Pedal input */

	uint8_t output_flags = 0u;
	if (CurtisIO_GetOutForward())  output_flags |= 0x01u;  /* bit0 FWD output */
	if (CurtisIO_GetOutBackward()) output_flags |= 0x02u;  /* bit1 REV output */
	if (CurtisIO_GetOutPedal())    output_flags |= 0x04u;  /* bit2 Pedal out  */
	if (CurtisIO_ModeRelayActive())output_flags |= 0x08u;  /* bit3 mode relay */

	/* Clamp both MCOR voltages to the uint16 mV range before packing. */
	float in_mv  = mcor_volts * 1000.0f;
	float out_mv = CurtisIO_McorOutVolts() * 1000.0f;
	if (in_mv  < 0.0f) in_mv  = 0.0f;  if (in_mv  > 65535.0f) in_mv  = 65535.0f;
	if (out_mv < 0.0f) out_mv = 0.0f;  if (out_mv > 65535.0f) out_mv = 65535.0f;
	uint16_t mcor_in_mv  = (uint16_t) (in_mv  + 0.5f);
	uint16_t mcor_out_mv = (uint16_t) (out_mv + 0.5f);

	float hz = speed_sensor_hz;
	if (hz < 0.0f) hz = 0.0f;  if (hz > 65535.0f) hz = 65535.0f;
	uint16_t sensor_hz = (uint16_t) (hz + 0.5f);

	uint8_t TxData[8];
	TxData[0] = input_flags;                             /* [0] input_flags   */
	TxData[1] = output_flags;                            /* [1] output_flags  */
	TxData[2] = (uint8_t) (mcor_in_mv & 0xFFu);          /* [2..3] MCOR in mV  */
	TxData[3] = (uint8_t) ((mcor_in_mv >> 8) & 0xFFu);
	TxData[4] = (uint8_t) (mcor_out_mv & 0xFFu);         /* [4..5] MCOR out mV */
	TxData[5] = (uint8_t) ((mcor_out_mv >> 8) & 0xFFu);
	TxData[6] = (uint8_t) (sensor_hz & 0xFFu);           /* [6..7] sensor Hz   */
	TxData[7] = (uint8_t) ((sensor_hz >> 8) & 0xFFu);

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CanSpeedDiagTxHeader, TxData);
}

/**
  * @brief FDCAN RX FIFO0 callback: decode /brake_command, /cmd_vel,
  *        /speed_enable and /servo_command frames.
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u) {
		return;
	}

	/* Drain every frame queued in FIFO0, not just one: a flash-erase stall can
	 * let several pile up, and reading only one would leave the rest unprocessed. */
	FDCAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];
	while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
		if (RxHeader.Identifier == CAN_ID_BRAKE_CMD) {
			/* std_msgs/Bool: data:true -> Relay ON (engage), data:false -> Relay OFF. */
			relay_cmd = (RxData[0] != 0u) ? 1u : 0u;
		} else if (RxHeader.Identifier == CAN_ID_CMD_VEL) {
			/* 2-byte signed int16, 0.01 m/s/LSB: PID target wheel speed (signed). */
			int16_t raw = (int16_t) ((uint16_t) RxData[0]
			                       | ((uint16_t) RxData[1] << 8));
			curtis_speed_target_mps = raw_to_mps(raw);
			/* Stamp the arrival for the /speed_status timeout flag (0x122 bit5). */
			last_cmd_vel_tick = HAL_GetTick();
			cmd_vel_seen      = 1u;
		} else if (RxHeader.Identifier == CAN_ID_SPEED_ENABLE) {
			/* std_msgs/Bool: 1 -> run the PID speed loop, 0 -> release outputs. */
			speed_control_enable = (RxData[0] != 0u) ? 1u : 0u;
		} else if (RxHeader.Identifier == CAN_ID_SERVO_CMD) {
			/* 8-byte: [0..3] start_deg, [4..7] stop_deg. Takes effect immediately;
			 * the flash persist is debounced in the main loop (FLASH_WRITE_QUIET_MS). */
			float new_start, new_stop;
			memcpy(&new_start, &RxData[0], sizeof(float));
			memcpy(&new_stop,  &RxData[4], sizeof(float));
			/* Clamp both to the valid servo range 0..180°. */
			if (new_start < 0.0f) new_start = 0.0f;
			if (new_start > 180.0f) new_start = 180.0f;
			if (new_stop  < 0.0f) new_stop  = 0.0f;
			if (new_stop  > 180.0f) new_stop  = 180.0f;
			if (new_start != start_angle_deg || new_stop != stop_angle_deg) {
				start_angle_deg = new_start;
				stop_angle_deg  = new_stop;
				flash_save_req  = 1;
				flash_req_tick  = HAL_GetTick();  /* restart the debounce window */
			}
		}
	}
}

/**
  * @brief TIM input-capture callback: speed sensor edges on TIM2 CH1 (PA15).
  *        Hands each captured counter value to the curtis_io speed tracker.
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		CurtisIO_SpeedOnCapture(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) // ทุกๆ 10ms
	{
		// 1. ทริกเกอร์ Watchdog
		HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);

		// 2a. LED2 rapid-blink (~10 Hz) as a flash-write ACK, for ~1 s after a
		//     verified write. Overrides LED2's normal heartbeat while active.
		if (led2_blink_ticks > 0) {
			led2_blink_ticks--;
			static uint8_t b = 0;
			if (++b >= LED2_BLINK_PERIOD) {
				b = 0;
				HAL_GPIO_TogglePin(GPIOB, LED2_Pin);
			}
			if (led2_blink_ticks == 0) {
				HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET); // leave LED2 in a known state
			}
		}

		// 2b. นับเวลาให้ครบ 1 วินาที (10ms * 100 = 1000ms) สำหรับ LED heartbeat
		led_counter++;
		if (led_counter >= 100) {
			HAL_GPIO_TogglePin(GPIOB, LED_1_Pin);
			if (led2_blink_ticks == 0) {
				HAL_GPIO_TogglePin(GPIOB, LED2_Pin); // normal LED2 heartbeat when not ACK-blinking
			}
			led_counter = 0;
		}

		// 3. ตั้ง Flag ส่งสถานะทุกๆ HEARTBEAT_TICKS (10ms). main loop สลับกลุ่ม
		//    brake/speed แต่ละครั้ง -> แต่ละกลุ่มถูกส่งทุก 20ms
		static uint16_t hb_counter = 0;
		if (++hb_counter >= HEARTBEAT_TICKS) {
			hb_counter = 0;
			can_tx_flag = 1;
		}

		// 4. ปุ่ม Toggle relay ที่ขา PC1 (active-high). สุ่มอ่านทุก 10ms = debounce
		//    สลับสถานะเฉพาะตอน "กดลง" (ขอบ released->pressed) เท่านั้น
		static GPIO_PinState btn_prev = GPIO_PIN_RESET; // released = low (pull-down)
		GPIO_PinState btn_now = HAL_GPIO_ReadPin(BTN_TOGGLE_GPIO_Port, BTN_TOGGLE_Pin);
		if (btn_prev == GPIO_PIN_RESET && btn_now == GPIO_PIN_SET) {
			relay_cmd = (relay_cmd == 1) ? 0u : 1u; // toggle
		}
		btn_prev = btn_now;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_2) // ตรวจสอบว่าเป็น Interrupt จากขา PD2
  {
    // 0. กรอง glitch: noise ตอนคอยล์ relay สวิตช์ทำให้ PD2 เกิด falling edge ปลอม.
    //    E-Stop จริงจะกดค้างให้สาย LOW ต่อเนื่อง ส่วน glitch จะเด้งกลับ HIGH ทันที.
    //    spin สั้นๆ แล้วอ่านซ้ำ ถ้ากลับเป็น HIGH = glitch -> ไม่ latch.
    for (volatile uint32_t i = 0; i < 4000u; i++) { __NOP(); }
    if (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) != GPIO_PIN_RESET) {
      __HAL_GPIO_EXTI_CLEAR_IT(ESTOP_Pin);  // ทิ้ง edge ปลอม
      return;
    }

    // 1. สั่งปิด Relay ทันทีที่ระดับฮาร์ดแวร์
    HAL_GPIO_WritePin(RELAY_Brake_GPIO_Port, RELAY_Brake_Pin, GPIO_PIN_RESET);

    // 2. บังคับเคลียร์ตัวแปรคำสั่งใน Live Expression
    // เพื่อป้องกันไม่ให้โค้ดใน while(1) กลับมาสั่งเปิด Relay อีก
    relay_cmd = 0;

    // 3. ตั้งสถานะ watchdog ให้เป็น Triggered เพื่อให้ PC รู้ว่าเกิด E-Stop
    watchdog_status = 1;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

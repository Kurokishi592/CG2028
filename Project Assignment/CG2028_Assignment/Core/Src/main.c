  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) CG2028 Teaching Team
  ******************************************************************************/

/* ---------------------------------------------------------------------------------------------------------- */
/* ------------------------------------------- Includes ----------------------------------------------------- */

/*
 * This file contains the main loop for sensor reading and preprocessing, fall detection, alerts and escalation, and communications
*/

// Standard library headers
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include <sys/stat.h>
#include <stdbool.h>

// Drivers and BSP headers for peripherals

#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.h"
#include "../../Drivers/BSP/Components/spirit1/SPIRIT1_Library/Inc/SPIRIT_Types.h"
#include "../../Drivers/BSP/Components/spirit1/SPIRIT1_Library/Inc/SPIRIT_PktBasic.h"
#include "../../Drivers/BSP/Components/spirit1/SPIRIT1_Library/Inc/SPIRIT_Irq.h"

// Private headers
#include "wifi.h"
#include "wifi_secrets.h"
#include "lcd.h"
#include "tones.h"
#include "fall_detection.h"
#include "sensors.h"

#ifdef DEBUG
#define FALL_DEBUG 1
#endif


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void init(void);
void blink_LED2(int delay_ms);
static void UART1_Init(void);
static int WIFI_AppSendText(const char *text);
extern void initialise_monitor_handles(void);	// for semi-hosting support (printf). Will not be required if transmitting via UART
void SPI_WIFI_ISR(void);
void SystemClock_Config(void);


static void Buzzer_Init(void);
static void Buzzer_PlayTone(uint16_t freq_hz);
static void Buzzer_Off(void);
static void update_alert_outputs(fall_event_t new_event); // Handles LED2 and Buzzer based on g_latched_event
static void telebot_task(uint32_t now, fall_event_t event); // Handles Wifi and telebot messages every new NEARFALL/REALFALL
static void oled_task(uint32_t now, sensors_t sensor_readings, fall_event_t event); // Handles OLED texts
static void button_task(uint32_t now); // Handles user button to clear REAL_FALL latch


UART_HandleTypeDef huart1;
static TIM_HandleTypeDef htim3;  // TIM3 CH1 drives passive buzzer PWM on ARD_D5 (PB4)

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == ISM43362_DRDY_EXTI1_Pin) {
		SPI_WIFI_ISR();
	}
}

void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
	blink_LED2(100);
  }
}

/* --------------------------- Globals and constants --------------------------- */
// Circular buffer to store most recent 4 accel readings
int i=0;												// Counter to keep track of how many readings have been taken

// Latched alert state for non-blocking LED/buzzer behavior
static fall_event_t g_latched_event = FALL_EVENT_NONE;
static uint32_t g_latched_timestamp = 0;

// Current classified event and when it was set (shared across tasks)
static fall_event_t g_current_event = FALL_EVENT_NONE;
static uint32_t g_current_event_timestamp = 0U;

// Whether the fall-detection state machine is allowed to run.
// After a REAL_FALL is detected, we pause it until the user
// acknowledges via the button.
static uint8_t g_detector_enabled = 1U;

// Sensor sampling period (controls how often we read sensors and run fall detection)
#define SENSOR_SAMPLE_PERIOD_MS 20U
static uint32_t g_last_sample_tick = 0U;

#define ALERT_TASK_PERIOD_MS 10U
#define BUTTON_TASK_PERIOD_MS 10U
#define TELEBOT_TASK_PERIOD_MS 100U
#define OLED_TASK_PERIOD_MS 50U
#define OLED_TASK_BUDGET_MS 200U
#define REAL_FALL_REPORT_PERIOD_MS 15000U

const char* NEAR_FALL_STR = "Near fall detected!";
const char* REAL_FALL_STR = "Real fall detected!";

static uint8_t g_wifi_server_ip[4] = {0};
static uint8_t g_wifi_ready = 0;
static const uint8_t g_wifi_socket = 1;
static const char *g_pending_alert_msg = NULL;
static uint8_t g_alert_pending = 0U;
static uint8_t g_real_fall_active = 0U;
static uint8_t g_real_fall_initial_pending = 0U;
static uint32_t g_real_fall_start_tick = 0U;
static uint32_t g_last_real_fall_report_tick = 0U;
static uint32_t g_next_real_fall_report_tick = 0U;
static uint8_t g_ack_pending = 0U;           // set when user presses button to acknowledge REAL_FALL
static uint32_t g_ack_fall_duration_s = 0U;   // elapsed seconds at time of acknowledgment

/* ---------------------------------------------------------------------------------------------------------- */

/*
 * This is the state machine of the fall detection algorithm.
 * Init -> Filter sensor accelerometer readings -> Calculate ||AT||, ||GT|| and angle <-> Check ||AT||>t_m -> (delta_AT>t_at && delta_GT>t_ga) -> check angle>t_th -> classify as real fall else near fall 
 */
int main(void)
{
	//-------------------------------------- Initialise Device --------------------------------------//
	init(); // initialize peripherals and UART for transmission

	static int csv_header_printed = 0;
	sensors_t sensor_readings;
	uint32_t last_alert_tick = 0U;
	uint32_t last_button_tick = 0U;
	uint32_t last_telebot_tick = 0U;
	uint32_t last_oled_tick = 0U;

	while (1)
	{
		uint32_t now = HAL_GetTick();
		fall_event_t fall_event = g_current_event; // snapshot for this loop

		// -------------------------------------- SENSOR SAMPLING + FALL DETECTION (TIMED) -------------------------------------- //
		if ((now - g_last_sample_tick) >= SENSOR_SAMPLE_PERIOD_MS)
		{
			g_last_sample_tick = now;

			/* -------------------------------------- FALL DETECTION -------------------------------------- */
			fall_event_t new_event = FALL_EVENT_NONE;
			get_IMU_reading(i, &sensor_readings);
			get_baro_reading(&sensor_readings);
			
			
			if (i >= 3 && g_detector_enabled)
			{
				new_event = detect_fall(sensor_readings.accel_magnitude_asm,
					sensor_readings.gyro_magnitude_asm,
					sensor_readings.accel_recent_range,
					sensor_readings.gyro_recent_range,
					sensor_readings.roll_pitch_yaw,
					sensor_readings.dp_hpa);
					
					if (new_event != g_current_event)
					{
						g_current_event = new_event;
						g_current_event_timestamp = now;
						fall_event = new_event;
						
						// Once we see a REAL_FALL, pause the detector until
						// the user acknowledges via the button.
						if (new_event == FALL_EVENT_REAL_FALL)
						{
							g_detector_enabled = 0U;
							g_real_fall_active = 1U;
							g_real_fall_initial_pending = 1U;
							g_real_fall_start_tick = now;
							g_last_real_fall_report_tick = 0U;
							g_next_real_fall_report_tick = now + REAL_FALL_REPORT_PERIOD_MS;
						}

						if (new_event == FALL_EVENT_NEAR_FALL)
						{
							g_pending_alert_msg = NEAR_FALL_STR;
							g_alert_pending = 1U;
						}
						else if (new_event == FALL_EVENT_REAL_FALL)
						{
							g_pending_alert_msg = NULL;
							g_alert_pending = 0U;
						}
					}
				}
			
			/* -------------------------------------- CSV DATA LOGGING OVER UART -------------------------------------- */
			#ifdef FALL_DEBUG
			char buffer[200];
			if(fall_event == FALL_EVENT_NEAR_FALL) {
				sprintf(buffer, "Classification: NEAR-FALL\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
			}
			else if(fall_event == FALL_EVENT_REAL_FALL) {
				sprintf(buffer, "Classification: REAL FALL\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
			}
			if (!csv_header_printed) {
				// Header row
				sprintf(buffer,
						"time_ms,acc,accR,gyro,gyroR,roll,pitch,yaw,pressure,dp_hpa,state,event\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
				csv_header_printed = 1;
			}

			// Data row
			sprintf(buffer,
					"%lu,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.2f,%.3f,%d,%d\r\n",
					(unsigned long)HAL_GetTick(),
					sensor_readings.accel_magnitude_asm,
					sensor_readings.accel_recent_range,
					sensor_readings.gyro_magnitude_asm,
					sensor_readings.gyro_recent_range,
					sensor_readings.roll_pitch_yaw[0],
					sensor_readings.roll_pitch_yaw[1],
					sensor_readings.roll_pitch_yaw[2],
					sensor_readings.pressure_hpa,
					sensor_readings.dp_hpa,
					get_fall_state(),
					(int)fall_event);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
			#endif
			i++;
		}

		// -------------------------------------- NON-BLOCKING TASKS (TIMESLICED) -------------------------------------- //
		if ((now - last_alert_tick) >= ALERT_TASK_PERIOD_MS)
		{
			last_alert_tick = now;
			update_alert_outputs(g_current_event);
		}

		if ((now - last_button_tick) >= BUTTON_TASK_PERIOD_MS)
		{
			last_button_tick = now;
			button_task(now);
		}

		if (g_ack_pending || (now - last_telebot_tick) >= TELEBOT_TASK_PERIOD_MS)
		{
			last_telebot_tick = now;
			telebot_task(now, g_latched_event);
		}

		if ((now - last_oled_tick) >= OLED_TASK_PERIOD_MS)
		{
			last_oled_tick = now;
			oled_task(now, sensor_readings, g_latched_event);
		}
	}
}

static void init (void)
{
	HAL_Init();													// Reset all peripherals, initialize flash interface and systick
	SystemClock_Config();
	UART1_Init();												// Initialize UART1 for serial communication
	fall_detection_init();										// Initialize fall detection state machine
	lcd_start();
	lcd_draw_text(80, 120, "Device", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(20, 160, "Initialization...", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);

	// Peripheral initializations using BSP functions
	BSP_LED_Init(LED2);
	// BSP_ACCELERO_Init();
	// BSP_PSENSOR_Init();
	// BSP_GYRO_Init();
	Buzzer_Init();
	// BSP_MAGNETO_Init();
	sensors_init();
	BSP_LED_Off(LED2);											// Set the initial LED state to off
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);					// Initialize user button
	
	// Initialize Wi-Fi module, connect to ESP32 
	char wifi_status_buf[100];
	
	int status_len_1 = sprintf(wifi_status_buf, "Device initialized\r\n");
	if (status_len_1 > 0) {
		HAL_UART_Transmit(&huart1, (uint8_t*)wifi_status_buf, (uint16_t)status_len_1, HAL_MAX_DELAY);
	}

	WIFI_Status_t wifi_status = WIFI_Init();

	wifi_status = WIFI_Connect(WIFI_SSID, WIFI_PASSWORD, WIFI_ECN_WPA2_PSK);
	lcd_clear(LCD_COLOR_WHITE);
	lcd_draw_text(65, 120, "Connecting", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(50, 160, "to Wi-Fi...", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	if (wifi_status != WIFI_STATUS_OK) {
		g_wifi_ready = 0;
		return;
	}
	unsigned int a = 0;
	unsigned int b = 0;
	unsigned int c = 0;
	unsigned int d = 0;
	char tail = '\0';
	if (sscanf(ESP32_PROXY_HOST, "%u.%u.%u.%u%c", &a, &b, &c, &d, &tail) == 4) {
		g_wifi_server_ip[0] = (uint8_t)a;
		g_wifi_server_ip[1] = (uint8_t)b;
		g_wifi_server_ip[2] = (uint8_t)c;
		g_wifi_server_ip[3] = (uint8_t)d;
	}
	
	wifi_status = WIFI_OpenClientConnection(g_wifi_socket, WIFI_TCP_PROTOCOL, "conn", g_wifi_server_ip, ESP32_PROXY_PORT, 0);
	if (wifi_status != WIFI_STATUS_OK) {
		g_wifi_ready = 0;
		return;
	} else {
		g_wifi_ready = 1;
	}
	lcd_clear(LCD_COLOR_GREEN);
	lcd_draw_text(65, 130, "Wi-Fi", LCD_COLOR_BLACK, LCD_COLOR_GREEN, 3);
	lcd_draw_text(30, 170, "Connected!", LCD_COLOR_BLACK, LCD_COLOR_GREEN, 3);
	HAL_Delay(500);
	lcd_clear(LCD_COLOR_WHITE);
	lcd_draw_text(75, 20, "Fall", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 4);
	lcd_draw_text(15, 60, "Detection", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 4);
	lcd_draw_text(50, 100, "Device", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 4);
	
	lcd_draw_text(35, 150, "Accel", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(30, 175, "00.00", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(150, 150, "Gyro", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(135, 175, "0000.00", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(25, 215, "Roll", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(25, 240, "00.0", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(90, 215, "Pitch", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(95, 240, "00.0", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(175, 215, "Yaw", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(165, 240, "00.0", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
}

static int WIFI_AppSendText(const char *text)
{
	const uint32_t WIFI_SEND_TIMEOUT_MS = 100U;

	if (text == NULL) {
		return -10;
	}

	uint8_t payload[128];
	int n = snprintf((char *)payload, sizeof(payload), "%s\n", text);
	if ((n <= 0) || (n >= (int)sizeof(payload))) {
		return -11;
	}

	// Always close and reopen the TCP connection before sending.
	// The ESP32 proxy closes each client after forwarding a message
	// (client.stop()), leaving the ISM43362 with a half-closed socket
	// that may silently accept data into its send buffer without
	// actually delivering it.  A fresh connection guarantees the
	// ESP32 will see the new message.
	(void)WIFI_CloseClientConnection(g_wifi_socket);
	WIFI_Status_t status = WIFI_OpenClientConnection(g_wifi_socket, WIFI_TCP_PROTOCOL, "conn", g_wifi_server_ip, ESP32_PROXY_PORT, 0);
	if (status != WIFI_STATUS_OK) {
		g_wifi_ready = 0;
		return -12;
	}
	g_wifi_ready = 1;

	uint16_t sent_len = 0;
	status = WIFI_SendData(g_wifi_socket, payload, (uint16_t)n, &sent_len, WIFI_SEND_TIMEOUT_MS);

	if ((status == WIFI_STATUS_OK) && (sent_len == (uint16_t)n)) {
		return 0;
	}
	return -13;
}



/* ---------- Passive buzzer on ARD_D5 (PB4) via TIM3 CH1 PWM ---------- */

static void Buzzer_Init(void)
{
	// Enable clocks
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// Configure PB4 as TIM3_CH1 alternate function (AF2)
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin       = ARD_D5_Pin;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(ARD_D5_GPIO_Port, &GPIO_InitStruct);

	// Base timer config (will be reconfigured per-tone in Buzzer_PlayTone)
	htim3.Instance               = TIM3;
	htim3.Init.Prescaler         = 79;            // 80 MHz / 80 = 1 MHz tick
	htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
	htim3.Init.Period            = 999;            // default ~1 kHz
	htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_PWM_Init(&htim3);

	// Configure channel 1 in PWM mode 1
	TIM_OC_InitTypeDef sConfig = {0};
	sConfig.OCMode     = TIM_OCMODE_PWM1;
	sConfig.Pulse      = 500;  // 50% duty (will be updated per-tone)
	sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfig, TIM_CHANNEL_1);
}

/**
 * Start (or change) the tone on the passive buzzer.
 * @param freq_hz  Desired frequency in Hz (use NOTE_xx defines from tones.h).
 */
static void Buzzer_PlayTone(uint16_t freq_hz)
{
	if (freq_hz == 0U) { Buzzer_Off(); return; }

	// TIM3 tick = 80 MHz / (PSC+1) = 1 MHz.  Period = 1 MHz / freq_hz.
	const uint32_t TIM_TICK = 1000000U;
	uint32_t period = TIM_TICK / (uint32_t)freq_hz;
	if (period == 0U) period = 1U;

	__HAL_TIM_SET_AUTORELOAD(&htim3, period - 1U);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, period / 2U);  // 50% duty
	__HAL_TIM_SET_COUNTER(&htim3, 0U);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

static void Buzzer_Off(void)
{
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

static void update_alert_outputs(fall_event_t new_event)
{
	uint32_t now = HAL_GetTick();

	// Latch events to avoid flicker; REAL_FALL stays latched
	// until cleared by the user button, NEAR_FALL auto-clears.
	// const uint32_t REAL_FALL_LATCH_MS = 0U;   // unused now
	const uint32_t NEAR_FALL_LATCH_MS = 2000U;

	if (new_event == FALL_EVENT_REAL_FALL)
	{
		g_latched_event = FALL_EVENT_REAL_FALL;
		g_latched_timestamp = now;
	}
	else if (new_event == FALL_EVENT_NEAR_FALL && g_latched_event == FALL_EVENT_NONE)
	{
		g_latched_event = FALL_EVENT_NEAR_FALL;
		g_latched_timestamp = now;
	}

	// Auto-clear only NEAR_FALL latches after timeout; REAL_FALL
	// is cleared by button_task() when the user acknowledges.
	if (g_latched_event == FALL_EVENT_NEAR_FALL && (now - g_latched_timestamp) > NEAR_FALL_LATCH_MS)
	{
		g_latched_event = FALL_EVENT_NONE;
	}

	// Non-blocking patterns for LED and buzzer
	static uint32_t last_buzz_toggle = 0U;
	static uint8_t  buzz_on = 0U;
	static uint32_t last_led_toggle = 0U;
	static uint8_t  led_on = 0U;
	static uint8_t  tone_on = 0U;  // is the passive buzzer currently playing?

	const uint32_t BUZZ_DELAY_MS     = 5000U; // buzzer escalates to beeping pattern 5s after REAL_FALL
	const uint32_t BUZZ_ON_MS        = 120U;  // beep-on duration
	const uint32_t BUZZ_OFF_MS       = 180U;  // beep-off duration
	const uint32_t LED_REAL_PERIODMS = 200U;  // fast blink for REAL_FALL
	const uint32_t LED_NEAR_PERIODMS = 600U;  // slower blink for NEAR_FALL
	const uint32_t NEAR_TONE_MS      = 500U;  // short beep duration for near-fall

	switch (g_latched_event)
	{
	case FALL_EVENT_REAL_FALL:
		// LED: fast blink
		if ((now - last_led_toggle) >= LED_REAL_PERIODMS)
		{
			if (led_on) { BSP_LED_Off(LED2); led_on = 0U; }
			else       { BSP_LED_On(LED2);  led_on = 1U; }
			last_led_toggle = now;
		}

		// Buzzer: continuous alarm immediately, then escalate to beep pattern after delay
		if ((now - g_latched_timestamp) < BUZZ_DELAY_MS)
		{
			// Continuous tone for first 5 seconds
			if (!tone_on) { Buzzer_PlayTone(NOTE_A5); tone_on = 1U; }
			break;
		}
		// After 5s: beep pattern (on/off) at a higher-pitch alarm
		if (!buzz_on)
		{
			if ((now - last_buzz_toggle) >= BUZZ_OFF_MS)
			{
				Buzzer_PlayTone(NOTE_A5);
				buzz_on = 1U;
				tone_on = 1U;
				last_buzz_toggle = now;
			}
			else if (tone_on)
			{
				Buzzer_Off();
				tone_on = 0U;
			}
		}
		else
		{
			if ((now - last_buzz_toggle) >= BUZZ_ON_MS)
			{
				Buzzer_Off();
				buzz_on = 0U;
				tone_on = 0U;
				last_buzz_toggle = now;
			}
		}
		break;

	case FALL_EVENT_NEAR_FALL:
		// NEAR FALL: slow LED blink, short warning beep
		if (!tone_on)
		{
			Buzzer_PlayTone(NOTE_C5);  // 523 Hz short warning
			tone_on = 1U;
		}
		// Auto-stop the tone after NEAR_TONE_MS
		if (tone_on && (now - g_latched_timestamp) >= NEAR_TONE_MS)
		{
			Buzzer_Off();
			tone_on = 0U;
		}

		if ((now - last_led_toggle) >= LED_NEAR_PERIODMS)
		{
			if (led_on) { BSP_LED_Off(LED2); led_on = 0U; }
			else       { BSP_LED_On(LED2);  led_on = 1U; }
			last_led_toggle = now;
		}
		break;

	case FALL_EVENT_NONE:
	default:
		// NO FALL: everything off
		if (tone_on) { Buzzer_Off(); tone_on = 0U; }
		buzz_on = 0U;
		BSP_LED_Off(LED2);
		led_on = 0U;
		break;
	}
}

// User button task: clears latched REAL_FALL on a *press*,
// not a long hold, but still debounces using the original
// "release to arm, then a few consecutive pressed samples"
// pattern. Non-blocking.
static void button_task(uint32_t now)
{
	(void)now; // not used, we debounce by counts instead of time
	static uint8_t  button_was_down = 0U;
	static uint8_t  button_reset_armed = 0U;
	static uint8_t  button_press_ticks = 0U;
	const uint8_t BUTTON_RESET_TICKS_REQUIRED = 10U;

	if (g_latched_event != FALL_EVENT_REAL_FALL)
	{
		button_was_down = 0U;
		button_reset_armed = 0U;
		button_press_ticks = 0U;
		return;
	}

	GPIO_PinState state = BSP_PB_GetState(BUTTON_USER);
	if (state == GPIO_PIN_SET)
	{
		if (!button_was_down)
		{
			button_was_down = 1U;
			button_press_ticks = 1U;
		}
		else if (button_reset_armed)
		{
			if (button_press_ticks < BUTTON_RESET_TICKS_REQUIRED)
			{
				button_press_ticks++;
			}
			if (button_press_ticks >= BUTTON_RESET_TICKS_REQUIRED)
			{
			// Record how long the fall lasted before acknowledgment
			if (g_real_fall_start_tick != 0U)
			{
				g_ack_fall_duration_s = (now - g_real_fall_start_tick) / 1000U;
			}
			else
			{
				g_ack_fall_duration_s = 0U;
			}
			g_ack_pending = 1U;  // signal telebot_task to send ack immediately

			// Clear latched REAL_FALL and re-arm detector
			g_latched_event = FALL_EVENT_NONE;
			g_current_event = FALL_EVENT_NONE;
			g_current_event_timestamp = now;
			g_real_fall_active = 0U;
			g_real_fall_initial_pending = 0U;
			g_real_fall_start_tick = 0U;
			g_last_real_fall_report_tick = 0U;
			g_next_real_fall_report_tick = 0U;
			fall_detection_init();
			g_detector_enabled = 1U;
			button_was_down = 0U;
			button_reset_armed = 0U;
			button_press_ticks = 0U;
			}
		}
	}
	else
	{
		button_was_down = 0U;
		button_press_ticks = 0U;
		// A clean release "arms" the reset; the next
		// short press that is stable for a couple of
		// samples will be treated as an
		// acknowledgement to clear REAL_FALL.
		button_reset_armed = 1U;
	}
}

// Telebot / Wi-Fi reporting task: send once per event change
static void telebot_task(uint32_t now, fall_event_t event)
{
	(void)event;
	static uint32_t last_attempt_tick = 0U;
	char real_fall_msg[96];
	uint32_t retry_interval_ms = g_wifi_ready ? 200U : 1000U;

	// Handle button acknowledgment message (highest priority, no delay)
	if (g_ack_pending)
	{
		uint32_t mins = g_ack_fall_duration_s / 60U;
		uint32_t secs = g_ack_fall_duration_s % 60U;
		snprintf(real_fall_msg, sizeof(real_fall_msg),
			"Button pressed, hard fall alert acknowledged. Fall lasted %lu min %lu sec",
			(unsigned long)mins, (unsigned long)secs);

		if (WIFI_AppSendText(real_fall_msg) == 0)
		{
			g_ack_pending = 0U;
			last_attempt_tick = now;
		}
		else if ((now - last_attempt_tick) >= retry_interval_ms)
		{
			// retry next cycle
			last_attempt_tick = now;
		}
		return;
	}

	if (g_real_fall_active)
	{
		uint8_t report_due = g_real_fall_initial_pending ||
			((now - g_next_real_fall_report_tick) < 0x80000000U);

		if (report_due)
		{
			if ((now - last_attempt_tick) < retry_interval_ms)
			{
				return;
			}

			last_attempt_tick = now;
			uint32_t elapsed_seconds = (now - g_real_fall_start_tick) / 1000U;
			uint32_t elapsed_minutes = elapsed_seconds / 60U;
			uint32_t remaining_seconds = elapsed_seconds % 60U;

			snprintf(real_fall_msg, sizeof(real_fall_msg),
				"Real fall detected. Time since fall: %lu min %lu sec",
				(unsigned long)elapsed_minutes,
				(unsigned long)remaining_seconds);

			if (WIFI_AppSendText(real_fall_msg) == 0)
			{
				g_last_real_fall_report_tick = now;
				if (g_real_fall_initial_pending)
				{
					g_real_fall_initial_pending = 0U;
				}
				else
				{
					do
					{
						g_next_real_fall_report_tick += REAL_FALL_REPORT_PERIOD_MS;
					} while ((now - g_next_real_fall_report_tick) < 0x80000000U);
				}
			}
			return;
		}
	}

	if (!g_alert_pending || g_pending_alert_msg == NULL)
	{
		return;
	}

	if ((now - last_attempt_tick) < retry_interval_ms)
	{
		return;
	}

	last_attempt_tick = now;
	if (WIFI_AppSendText(g_pending_alert_msg) == 0)
	{
		g_alert_pending = 0U;
		g_pending_alert_msg = NULL;
	}
}

// OLED update task: update display only when classification changes
static void oled_task(uint32_t now, sensors_t sensor_readings, fall_event_t event)
{
	uint32_t slice_start = HAL_GetTick();
	(void)now;
	(void)event;

	char acc_str[24];
	char gyro_str[24];
	char roll_str[12];
	char pitch_str[12];
	char yaw_str[12];
	static bool updateState = false;
	static fall_event_t prevEvent = FALL_EVENT_NONE;
	static uint16_t bg_color = LCD_COLOR_WHITE;

	snprintf(acc_str, sizeof(acc_str), "%05.2f", sensor_readings.accel_magnitude_asm);
	snprintf(gyro_str, sizeof(gyro_str), "%05.2f", sensor_readings.gyro_magnitude_asm);
	snprintf(roll_str, sizeof(roll_str), "%.1f", sensor_readings.roll_pitch_yaw[0]);
	snprintf(pitch_str, sizeof(pitch_str), "%.1f", sensor_readings.roll_pitch_yaw[1]);
	snprintf(yaw_str, sizeof(yaw_str), "%.1f", sensor_readings.roll_pitch_yaw[2]);

	
	lcd_draw_text(35, 150, "Accel", LCD_COLOR_BLACK, bg_color, 2);
	lcd_draw_text(30, 175, acc_str, LCD_COLOR_BLACK, bg_color, 2);
	if ((HAL_GetTick() - slice_start) >= OLED_TASK_BUDGET_MS) return;
	lcd_draw_text(150, 150, "Gyro", LCD_COLOR_BLACK, bg_color, 2);
	lcd_draw_text(135, 175, gyro_str, LCD_COLOR_BLACK, bg_color, 2);
	if ((HAL_GetTick() - slice_start) >= OLED_TASK_BUDGET_MS) return;
	lcd_draw_text(25, 215, "Roll", LCD_COLOR_BLACK, bg_color, 2);
	lcd_draw_text(20, 240, roll_str, LCD_COLOR_BLACK, bg_color, 2);
	if ((HAL_GetTick() - slice_start) >= OLED_TASK_BUDGET_MS) return;
	lcd_draw_text(90, 215, "Pitch", LCD_COLOR_BLACK, bg_color, 2);
	lcd_draw_text(95, 240, pitch_str, LCD_COLOR_BLACK, bg_color, 2);
	if ((HAL_GetTick() - slice_start) >= OLED_TASK_BUDGET_MS) return;
	lcd_draw_text(175, 215, "Yaw", LCD_COLOR_BLACK, bg_color, 2);
	lcd_draw_text(165, 240, yaw_str, LCD_COLOR_BLACK, bg_color, 2);

	if (event == FALL_EVENT_NEAR_FALL && updateState)
	{
		bg_color = LCD_COLOR_YELLOW;
		lcd_clear(bg_color);
		lcd_draw_text(75, 20, "Fall", LCD_COLOR_BLACK, bg_color, 4);
		lcd_draw_text(15, 60, "Detection", LCD_COLOR_BLACK, bg_color, 4);
		lcd_draw_text(50, 100, "Device", LCD_COLOR_BLACK, bg_color, 4);
		updateState = false;	
	}
	else if (event == FALL_EVENT_REAL_FALL && updateState)
	{
		bg_color = LCD_COLOR_RED;
		lcd_clear(bg_color);
		lcd_draw_text(70, 20, "Haha", LCD_COLOR_BLACK, bg_color, 4);
		lcd_draw_text(80, 60, "You", LCD_COLOR_BLACK, bg_color, 4);
		lcd_draw_text(70, 100, "Fell!", LCD_COLOR_BLACK, bg_color, 4);
		lcd_draw_text(50, 280, "Press button", LCD_COLOR_WHITE, bg_color, 2);
		lcd_draw_text(65, 300, "to revive", LCD_COLOR_WHITE, bg_color, 2);		
		updateState = false;
	}
	else if (updateState) {
		bg_color = LCD_COLOR_WHITE;
		lcd_clear(bg_color);
		lcd_draw_text(75, 20, "Fall", LCD_COLOR_BLACK, bg_color, 4);
		lcd_draw_text(15, 60, "Detection", LCD_COLOR_BLACK, bg_color, 4);
		lcd_draw_text(50, 100, "Device", LCD_COLOR_BLACK, bg_color, 4);		
		updateState = false;
	}

	if (event != prevEvent)
	{
		updateState = true;
		prevEvent = event;
	}
}

/**
 * @brief  UART1 Initialization Function
 * Pin configuration for UART. BSP_COM_Init() can do this automatically.
 */
static void UART1_Init(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Configuring UART1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		while(1);
	}

}

void SystemClock_Config(void)
{
  /* oscillator and clocks configs */
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  
  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Disable Power Control clock */
  __HAL_RCC_PWR_CLK_DISABLE();

  /* 80 Mhz from PLL with MSI 8Mhz as source clock */
  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;   /* 8 Mhz */
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * Helper function for LED blinking
 * @param delay_ms: Delay in milliseconds between toggles
 */
void blink_LED2(int delay_ms)
{
	BSP_LED_Toggle(LED2);
	HAL_Delay(delay_ms);
}


// Do not modify these lines of code. They are written to supress UART related warnings
int _read(int file, char *ptr, int len) { return 0; }
int _fstat(int file, struct stat *st) { return 0; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _isatty(int file) { return 1; }
int _close(int file) { return -1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { return -1; }

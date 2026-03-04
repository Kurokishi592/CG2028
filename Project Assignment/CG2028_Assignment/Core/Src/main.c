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
#define FALL_DEBUG 0
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void init(void);
static void UART1_Init(void);
static int WIFI_AppSendText(const char *text);
extern void initialise_monitor_handles(void);
void SPI_WIFI_ISR(void);
void SystemClock_Config(void);


static void Buzzer_Init(void);
static void Buzzer_On(void);
static void Buzzer_Off(void);
static void update_alert_outputs(fall_event_t new_event); 								// Handles LED2 and Buzzer based on g_latched_event
static void telebot_task(uint32_t now, fall_event_t event); 							// Handles Wifi and telebot messages every new NEARFALL/REALFALL
static void oled_task(uint32_t now, sensors_t sensor_readings, fall_event_t event); 	// Handles OLED texts
static void button_task(uint32_t now); 													// Handles user button to clear REAL_FALL latch


UART_HandleTypeDef huart1;				// UART handler for debug printing to Serial Monitor

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == ISM43362_DRDY_EXTI1_Pin)
	{
		SPI_WIFI_ISR();
	}
}

void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
	BSP_LED_Toggle(LED2);
	HAL_Delay(100);
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

const char* NEAR_FALL_STR = "Sudden movement detected!";
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
static uint8_t g_ack_pending = 0U;           		// set when user presses button to acknowledge REAL_FALL
static uint32_t g_ack_fall_duration_s = 0U;   		// elapsed seconds at time of acknowledgment

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
		fall_event_t fall_event = g_current_event;	// snapshot for this loop

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
						
						// Upon REAL_FALL, pause detector until user acknowledges via the button.
						if (new_event == FALL_EVENT_REAL_FALL)
						{
							g_detector_enabled = 0U;												// Disable fall detection until acknowledgment
							g_real_fall_active = 1U;												// Set REAL_FALL active flag to signify state
							g_real_fall_initial_pending = 1U;										// Set pending flag to trigger immediate alert for REAL_FALL
							g_real_fall_start_tick = now;											// Start timing REAL_FALL duration from now
							g_last_real_fall_report_tick = 0U;										// Report REAL_FALL at time now
							g_next_real_fall_report_tick = now + REAL_FALL_REPORT_PERIOD_MS;		// Set next report time
						}
						
						// Upon NEAR_FALL, send message, set alert pending flag
						if (new_event == FALL_EVENT_NEAR_FALL)
						{
							g_pending_alert_msg = NEAR_FALL_STR;
							g_alert_pending = 1U;
						}
						// If a REAL_FALL is detected during NEAR_FALL, change state to REAL_FALL instead
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

		/* -------------------------------------- NON-BLOCKING TASKS (TIMESLICED) ------------------------------------- */
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

/**
 * @brief  Initialize sensors, peripherals and display
 */
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
	Buzzer_Init();
	sensors_init();
	BSP_LED_Off(LED2);											// Set the initial LED state to off
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);					// Initialize user button
	
	// Initialize Wi-Fi module, connect to ESP32 - uncomment for debugging
	// char wifi_status_buf[100];
	
	// int status_len_1 = sprintf(wifi_status_buf, "Device initialized\r\n");
	// if (status_len_1 > 0) {
	// 	HAL_UART_Transmit(&huart1, (uint8_t*)wifi_status_buf, (uint16_t)status_len_1, HAL_MAX_DELAY);
	// }

	// Connect to Wi-Fi network 
	WIFI_Status_t wifi_status = WIFI_Init();

	wifi_status = WIFI_Connect(WIFI_SSID, WIFI_PASSWORD, WIFI_ECN_WPA2_PSK);
	lcd_clear(LCD_COLOR_WHITE);
	lcd_draw_text(65, 120, "Connecting", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(50, 160, "to Wi-Fi...", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	if (wifi_status != WIFI_STATUS_OK)
	{
		g_wifi_ready = 0;
		return;
	}

	// Connect to ESP32 proxy server, with fixed IP address
	unsigned int a = 0;
	unsigned int b = 0;
	unsigned int c = 0;
	unsigned int d = 0;
	char tail = '\0';
	if (sscanf(ESP32_PROXY_HOST, "%u.%u.%u.%u%c", &a, &b, &c, &d, &tail) == 4)
	{
		g_wifi_server_ip[0] = (uint8_t)a;
		g_wifi_server_ip[1] = (uint8_t)b;
		g_wifi_server_ip[2] = (uint8_t)c;
		g_wifi_server_ip[3] = (uint8_t)d;
	}
	
	wifi_status = WIFI_OpenClientConnection(g_wifi_socket, WIFI_TCP_PROTOCOL, "conn", g_wifi_server_ip, ESP32_PROXY_PORT, 0);
	if (wifi_status != WIFI_STATUS_OK)
	{
		g_wifi_ready = 0;
		return;
	} else
	{
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
	lcd_draw_text(20, 215, "Roll", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(15, 240, "00.0", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(85, 215, "Pitch", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(90, 240, "00.0", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(165, 215, "Baro", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
	lcd_draw_text(155, 240, "0000.0", LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2);
}

/**
 * @brief Send text to ESP32, which will be forwarded to the Telegram bot
 */
static int WIFI_AppSendText(const char *text)
{
	const uint32_t WIFI_SEND_TIMEOUT_MS = 100U;

	if (text == NULL)
	{
		return -10;
	}

	uint8_t payload[128];
	int n = snprintf((char *)payload, sizeof(payload), "%s\n", text);
	if ((n <= 0) || (n >= (int)sizeof(payload)))
	{
		return -11;
	}

	// Always close and reopen the TCP connection before sending
	// The ESP32 proxy closes each client after forwarding a message
	// (client.stop()), leaving the ISM43362 with a half-closed socket
	// that may silently accept data into its send buffer without
	// actually delivering it, a fresh connection guarantees the
	// ESP32 will see the new message
	(void)WIFI_CloseClientConnection(g_wifi_socket);
	WIFI_Status_t status = WIFI_OpenClientConnection(g_wifi_socket, WIFI_TCP_PROTOCOL, "conn", g_wifi_server_ip, ESP32_PROXY_PORT, 0);
	if (status != WIFI_STATUS_OK)
	{
		g_wifi_ready = 0;
		return -12;
	}
	g_wifi_ready = 1;

	uint16_t sent_len = 0;
	status = WIFI_SendData(g_wifi_socket, payload, (uint16_t)n, &sent_len, WIFI_SEND_TIMEOUT_MS);

	if ((status == WIFI_STATUS_OK) && (sent_len == (uint16_t)n))
	{
		return 0;
	}
	return -13;
}

/**
 * @brief Initialize buzzer GPIO pin, set initial state to off
 */
static void Buzzer_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	// Use ARD_D5 as a generic buzzer output pin (active-high)
	GPIO_InitStruct.Pin = ARD_D5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ARD_D5_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(ARD_D5_GPIO_Port, ARD_D5_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Helper function to turn on the buzzer
 */
static void Buzzer_On(void)
{
	HAL_GPIO_WritePin(ARD_D5_GPIO_Port, ARD_D5_Pin, GPIO_PIN_SET);
}

/**
 * @brief Helper function to turn off the buzzer
 */
static void Buzzer_Off(void)
{
	HAL_GPIO_WritePin(ARD_D5_GPIO_Port, ARD_D5_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Update alert outputs (LED2 and buzzer) based on the current latched event state
 * Implements non-blocking behavior for buzzer patterns
 * @param new_event Newly classified fall event to update outputs with
 */
static void update_alert_outputs(fall_event_t new_event)
{
	uint32_t now = HAL_GetTick();

	// Latch events to avoid hysteresis; REAL_FALL stays latched until
	// cleared by the user button, NEAR_FALL auto-clears if no REAL_FALL occurs.
	const uint32_t NEAR_FALL_LATCH_MS = 2000U;

	if (new_event == FALL_EVENT_REAL_FALL)
	{
		if (g_latched_event != FALL_EVENT_REAL_FALL)			// Safeguard to prevent resetting REAL_FALL timer
		{
			g_latched_event = FALL_EVENT_REAL_FALL;
			g_latched_timestamp = now;
		}
	}
	else if (new_event == FALL_EVENT_NEAR_FALL && g_latched_event == FALL_EVENT_NONE)
	{
		g_latched_event = FALL_EVENT_NEAR_FALL;
		g_latched_timestamp = now;
	}

	// Automatic clear of NEAR_FALL latch after timeout
	if (g_latched_event == FALL_EVENT_NEAR_FALL && (now - g_latched_timestamp) > NEAR_FALL_LATCH_MS)
	{
		g_latched_event = FALL_EVENT_NONE;
	}

	// Non-blocking patterns for LED and buzzer
	static uint32_t last_buzz_toggle = 0U;
	static uint8_t  buzz_on = 0U;
	static uint32_t last_led_toggle = 0U;
	static uint8_t  led_on = 0U;

	const uint32_t BUZZ_DELAY_MS     = 5000U;  // 5s silence period
	const uint32_t BUZZ_ON_MS        = 120U;   // beep-on duration (constant)
	const uint32_t BUZZ_OFF_SLOW_MS  = 400U;   // slowest beep (5-15s)
	const uint32_t BUZZ_OFF_MID_MS   = 200U;   // middle beep  (15-25s)
	const uint32_t BUZZ_OFF_FAST_MS  = 80U;    // fastest beep   (>25s)
	const uint32_t BUZZ_STEP_MS      = 10000U; // 10s between each change
	const uint32_t LED_REAL_PERIODMS = 200U;   // fast blink for REAL_FALL
	const uint32_t LED_NEAR_PERIODMS = 600U;   // slower blink for NEAR_FALL

	switch (g_latched_event)
	{
	case FALL_EVENT_REAL_FALL:
	// REAL FALL: fast LED blink + buzzer
	{
		uint32_t elapsed = now - g_latched_timestamp;

		// LED: fast blink
		if ((now - last_led_toggle) >= LED_REAL_PERIODMS)
		{
			if (led_on) { BSP_LED_Off(LED2); led_on = 0U; }
			else       { BSP_LED_On(LED2);  led_on = 1U; }
			last_led_toggle = now;
		}

		// Buzzer: silent for first 5s, then beep with accelerating rate
		if (elapsed < BUZZ_DELAY_MS)
		{
			Buzzer_Off();
			buzz_on = 0U;
			break;
		}

		// Calculate current off-gap based on which 10s step we're in
		uint32_t beep_elapsed = elapsed - BUZZ_DELAY_MS;
		uint32_t current_off;
		if (beep_elapsed < BUZZ_STEP_MS)
			current_off = BUZZ_OFF_SLOW_MS;			// 5-15s: slowest
		else if (beep_elapsed < 2U * BUZZ_STEP_MS)
			current_off = BUZZ_OFF_MID_MS;			// 15-25s: middle
		else
			current_off = BUZZ_OFF_FAST_MS;			// >25s: fastest

		if (buzz_on)
		{
			if ((now - last_buzz_toggle) >= BUZZ_ON_MS)
			{
				Buzzer_Off();
				buzz_on = 0U;
				last_buzz_toggle = now;
			}
		}
		else
		{
			if ((now - last_buzz_toggle) >= current_off)
			{
				Buzzer_On();
				buzz_on = 1U;
				last_buzz_toggle = now;
			}
		}
		break;
	}

	case FALL_EVENT_NEAR_FALL:
		// NEAR FALL: slow LED blink, no buzzer
		Buzzer_Off();
		buzz_on = 0U;

		if ((now - last_led_toggle) >= LED_NEAR_PERIODMS)
		{
			if (led_on)
			{
				BSP_LED_Off(LED2);
				led_on = 0U;
			} else
			{
				BSP_LED_On(LED2);
				led_on = 1U;
			}
			last_led_toggle = now;
		}
		break;

	case FALL_EVENT_NONE:
	default:
		// NO FALL: everything off
		Buzzer_Off();
		buzz_on = 0U;
		BSP_LED_Off(LED2);
		led_on = 0U;
		break;
	}
}

/**
 * @brief Task to handle user button presses to acknowledge REAL_FALL events
 * Implements a debounce mechanism and requires a short press (not long hold) to acknowledge
 * Calculates how long the REAL_FALL lasted before acknowledgment
 * @param now Current tick count for timing purposes
 */
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
		button_reset_armed = 1U;	// Re-arm reset on next press after release
	}
}

/**
 * @brief Task to handle sending messages to the ESP32 proxy for Telegram bot forwarding
 * Prioritizes acknowledgment messages, then REAL_FALL reports, then NEAR_FALL reports
 * @param now Current tick count for timing purposes
 * @param event The most recently classified fall event
 */
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
		else if ((now - last_attempt_tick) >= retry_interval_ms)		// Retry next cycle if message did not send
		{
			last_attempt_tick = now;
		}
		return;
	}

	// Handle REAL_FALL state - immediate report, then every REAL_FALL_REPORT_PERIOD_MS while still active
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

	// If no message to send, free up CPU by returning early
	if (!g_alert_pending || g_pending_alert_msg == NULL)
	{
		return;
	}

	// For REAL_FALL, if message recently sent, wait for retry interval before sending next
	if ((now - last_attempt_tick) < retry_interval_ms)
	{
		return;
	}

	// Attempt to send pending g_pending_alert_msg, clears g_alert_pending flag if successful
	last_attempt_tick = now;
	if (WIFI_AppSendText(g_pending_alert_msg) == 0)
	{
		g_alert_pending = 0U;
		g_pending_alert_msg = NULL;
	}
}

/**
 * @brief Task to handle updating the OLED display with sensor readings and fall event information
 * Prioritizes showing REAL_FALL state, then NEAR_FALL, then sensor readings
 * @param now Current tick count for timing purposes
 * @param sensor_readings The most recent sensor readings to display
 * @param event The most recently classified fall event for background color
 */
static void oled_task(uint32_t now, sensors_t sensor_readings, fall_event_t event)
{
	uint32_t slice_start = HAL_GetTick();
	(void)now;
	(void)event;

	char acc_str[24];
	char gyro_str[24];
	char roll_str[12];
	char pitch_str[12];
	char baro_str[12];
	static bool updateState = false;
	static fall_event_t prevEvent = FALL_EVENT_NONE;
	static uint16_t bg_color = LCD_COLOR_WHITE;

	snprintf(acc_str, sizeof(acc_str), "%05.2f", sensor_readings.accel_magnitude_asm);
	snprintf(gyro_str, sizeof(gyro_str), "%05.2f", sensor_readings.gyro_magnitude_asm);
	snprintf(roll_str, sizeof(roll_str), "%.1f", sensor_readings.roll_pitch_yaw[0]);
	snprintf(pitch_str, sizeof(pitch_str), "%.1f", sensor_readings.roll_pitch_yaw[1]);
	snprintf(baro_str, sizeof(baro_str), "%.1f", sensor_readings.pressure_hpa);

	
	lcd_draw_text(35, 150, "Accel", LCD_COLOR_BLACK, bg_color, 2);
	lcd_draw_text(30, 175, acc_str, LCD_COLOR_BLACK, bg_color, 2);
	if ((HAL_GetTick() - slice_start) >= OLED_TASK_BUDGET_MS) return;
	lcd_draw_text(150, 150, "Gyro", LCD_COLOR_BLACK, bg_color, 2);
	lcd_draw_text(135, 175, gyro_str, LCD_COLOR_BLACK, bg_color, 2);
	if ((HAL_GetTick() - slice_start) >= OLED_TASK_BUDGET_MS) return;
	lcd_draw_text(20, 215, "Roll", LCD_COLOR_BLACK, bg_color, 2);
	lcd_draw_text(15, 240, roll_str, LCD_COLOR_BLACK, bg_color, 2);
	if ((HAL_GetTick() - slice_start) >= OLED_TASK_BUDGET_MS) return;
	lcd_draw_text(85, 215, "Pitch", LCD_COLOR_BLACK, bg_color, 2);
	lcd_draw_text(90, 240, pitch_str, LCD_COLOR_BLACK, bg_color, 2);
	if ((HAL_GetTick() - slice_start) >= OLED_TASK_BUDGET_MS) return;
	lcd_draw_text(165, 215, "Baro", LCD_COLOR_BLACK, bg_color, 2);
	lcd_draw_text(155, 240, baro_str, LCD_COLOR_BLACK, bg_color, 2);

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

/**
 * @brief  System Clock Configuration
 */
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

// Do not modify these lines of code. They are written to supress UART related warnings
int _read(int file, char *ptr, int len) { return 0; }
int _fstat(int file, struct stat *st) { return 0; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _isatty(int file) { return 1; }
int _close(int file) { return -1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { return -1; }

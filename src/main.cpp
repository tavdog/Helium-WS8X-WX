/**
 * @file main.cpp
 * @author Bernd Giesecke (bernd@giesecke.tk)
 * @brief Example for WisBlock Tool Box to setup LoRa/LoRaWAN
 * @version 0.1
 * @date 2025-04-02
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "main.h"
#include "ws8x.h"

#define LORAWAN_APP_DATA_BUFF_SIZE 64
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0};

static bool initialSendDone = false;
static int send_error_count = 0;

// Main Interval
//   _____ _   _ _______ ______ _______      __     _
//  |_   _| \ | |__   __|  ____|  __ \ \    / /\   | |
//    | | |  \| |  | |  | |__  | |__) \ \  / /  \  | |
//    | | | . ` |  | |  |  __| |  _  / \ \/ / /\ \ | |
//   _| |_| |\  |  | |  | |____| | \ \  \  / ____ \| |____
//  |_____|_| \_|  |_|  |______|_|  \_\  \/_/    \_\______|

#define SEND_INTERVAL 1 // minutes

// Timer for sending data
unsigned long lastSendTime = 0;
unsigned long send_interval_ms = SEND_INTERVAL * 60000;

/** Flag for the event type */
volatile uint16_t g_task_event_type = NO_EVENT;

/** Set the device name, max length is 10 characters */
char g_ble_dev_name[10] = "WS8X";

/** Set firmware version */
uint16_t g_sw_ver_1 = 1; // major version increase on API change / not backwards compatible
uint16_t g_sw_ver_2 = 0; // minor version increase on API change / backward compatible
uint16_t g_sw_ver_3 = 0; // patch version increase on bugfix, no affect on API

/** Timer for frequent packet sending */
time_t last_send;

/**
 * @brief Arduino setup, called once
 *
 */
void setup(void)
{
	// flash_reset();
	// Initialize the built in LED
	pinMode(LED_GREEN, OUTPUT);
	digitalWrite(LED_GREEN, LOW);

	// Initialize the BLE status LED
	pinMode(LED_BLUE, OUTPUT);
	digitalWrite(LED_BLUE, LOW);

	// Initialize Serial
	Serial.begin(115200);

	time_t serial_timeout = millis();
	// On nRF52840 the USB serial is not available immediately
	while (!Serial)
	{
		if ((millis() - serial_timeout) < 5000)
		{
			delay(100);
			digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
		}
		else
		{
			break;
		}
	}
	digitalWrite(LED_GREEN, HIGH);

	// Get LoRa parameter
	init_flash();

	// Enable BLE
	APP_LOG("SETUP", "Init BLE");

	// Init BLE
	init_ble();

	// If P2P mode, override auto join setting
	if (!g_lorawan_settings.lorawan_enable)
	{
		g_lorawan_settings.auto_join = true;
	}

	// LoRaWAN initialization
	// Check if auto join is enabled
	if (g_lorawan_settings.auto_join || 1)
	{
		// Initialize LoRa and start join request
		int8_t lora_init_result = 0;
		if (g_lorawan_settings.lorawan_enable)
		{
			APP_LOG("SETUP", "Auto join is enabled, start LoRaWAN and join");
			lora_init_result = init_lorawan();
		}
		else
		{
			APP_LOG("API", "Auto join is enabled, start LoRa P2P listen");
			lora_init_result = init_lora();
		}

		if (lora_init_result != 0)
		{
			APP_LOG("SETUP", "Init LoRa failed");
			APP_LOG("SETUP", "Get your LoRa stuff in order");
		}
		else
		{
			APP_LOG("SETUP", "LoRa init success");
		}
	}
	else
	{
		// Put radio into sleep mode
		lora_rak4630_init();
		Radio.Sleep();

		APP_LOG("SETUP", "Auto join is disabled, waiting for connect command");
		delay(100);
	}

	// Keep BLE advertising forever
	restart_advertising(0);

	// Set time for sending a packet
	last_send = millis();
	if (g_lorawan_settings.send_repeat_time == 0) {
		APP_LOG("SETUP", "setting interval to default:  %d sec", send_interval_ms/1000);
		g_lorawan_settings.send_repeat_time = send_interval_ms;
	} else {
		APP_LOG("SETUP", "interval set to:  %d sec", g_lorawan_settings.send_repeat_time/1000 );
		}

		ws8x_init();
}

/**
 * @brief Arduino loop
 *
 */
void loop(void)
{
	// BLE UART event (AT commands)
	if ((g_task_event_type & BLE_DATA) == BLE_DATA)
	{
		g_task_event_type &= N_BLE_DATA;
		// Send it to AT command parser
		while (g_ble_uart.available() > 0)
		{
			at_serial_input(uint8_t(g_ble_uart.read()));
			delay(5);
		}
		at_serial_input(uint8_t('\n'));
	}

	// BLE config characteristic received
	if ((g_task_event_type & BLE_CONFIG) == BLE_CONFIG)
	{
		g_task_event_type &= N_BLE_CONFIG;
		APP_LOG("LOOP", "Config received over BLE");
		delay(100);

		// Inform connected device about new settings
		g_lora_data.write((void *)&g_lorawan_settings, sizeof(s_lorawan_settings));
		g_lora_data.notify((void *)&g_lorawan_settings, sizeof(s_lorawan_settings));
	}

	// Serial input event (AT commands)
	if ((g_task_event_type & AT_CMD) == AT_CMD)
	{
		g_task_event_type &= N_AT_CMD;
		while (Serial.available() > 0)
		{
			at_serial_input(uint8_t(Serial.read()));
			delay(5);
		}
	}

	ws8x_checkSerial();
	// if time to send.  if initialsend yet to happen use interim interval of 60 seconds.
	if (millis() - lastSendTime >= g_lorawan_settings.send_repeat_time || (!initialSendDone && millis() - lastSendTime >= 60000 ))
	{
		if (lmh_join_status_get() == LMH_SET)
		{

			lastSendTime = millis();

			// After first send, switch to normal interval AT+SENDINT=
			if (!initialSendDone)
			{
				initialSendDone = true;
				Serial.printf("Switching to normal send interval: %lu minutes\n", g_lorawan_settings.send_repeat_time / 60000);
			}

			ws8x_populate_lora_buffer(&m_lora_app_data, LORAWAN_APP_DATA_BUFF_SIZE);

			m_lora_app_data.port = LORAWAN_APP_PORT;
			lmh_error_status error;
			int retryCount = 0;
			const int maxRetries = 5;
			do
			{
				error = lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
				if (error == LMH_SUCCESS)
				{
					Serial.println("LoRa data sent successfully.");
					send_error_count = 0;  // reset the error_count on success.
					ws8x_reset_counters(); // reset the averaging counters.
					break;				   // Exit the loop if the send is successful
				}
				else
				{
					retryCount++;
					Serial.printf("lmh_send failed with error code: %d\n", error);
					Serial.printf("LoRa data send failed. Attempt %d of %d\n", retryCount, maxRetries);
					delay(1000); // Optional: Add a delay between retries
				}
			} while (retryCount < maxRetries);

			if (retryCount == maxRetries)
			{
				Serial.println("LoRa data send failed after maximum retries.");
				send_error_count++; // reset the error_count on success.
				if (send_error_count > 5)
				{
					// reboot.
					Serial.println("5 Cycles of send error, Rebooting");
					NVIC_SystemReset(); // Perform a system reset
				}
			}
		}
		else
		{
			Serial.println("Not joined to the network. Cannot send data.");
			delay(1000);
			send_error_count++;
			Serial.printf("send_error_count : %d", send_error_count);
			if (send_error_count > 5)
			{
				// reboot.
				Serial.println("No Connection, Rebooting");
				NVIC_SystemReset(); // Perform a system reset
			}
		}
	}

	yield();
}

uint8_t boardGetBatteryLevel(void)
{
	float voltage = (analogRead(BATTERY_PIN) * REAL_VBAT_MV_PER_LSB) / 1000.0; // Convert to volts

	// Define voltage range for battery level
	const float MAX_VOLTAGE = 4.2; // Maximum LiPo voltage
	const float MIN_VOLTAGE = 3.0; // Minimum LiPo voltage

	if (voltage > MAX_VOLTAGE)
	{
		return 254; // Max level (254 as per LoRaWAN spec)
	}
	else if (voltage < MIN_VOLTAGE)
	{
		return 1; // Min level (1 as per LoRaWAN spec)
	}

	// Calculate percentage (1-254 range)
	uint8_t level = 1 + ((voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 253;
	return level;
	// float raw;

	// // Get the raw 12-bit, 0..3000mV ADC value
	// raw = analogRead(BATTERY_PIN);

	// // Convert the raw value to compensated mv, taking the resistor-
	// // divider into account (providing the actual LIPO voltage)
	// // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
	// return (uint8_t)(raw * REAL_VBAT_MV_PER_LSB *2.55);
}
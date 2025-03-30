/**
 * @file LoRaWAN_OTAA.ino
 * @author rakwireless.com
 * @brief LoRaWan node example with OTAA registration
 * @version 0.1
 * @date 2020-08-21
 * 
 * @copyright Copyright (c) 2020
 * 
 * @note RAK5005-O GPIO mapping to RAK4631 GPIO ports
   RAK5005-O <->  nRF52840
   IO1       <->  P0.17 (Arduino GPIO number 17)
   IO2       <->  P1.02 (Arduino GPIO number 34)
   IO3       <->  P0.21 (Arduino GPIO number 21)
   IO4       <->  P0.04 (Arduino GPIO number 4)
   IO5       <->  P0.09 (Arduino GPIO number 9)
   IO6       <->  P0.10 (Arduino GPIO number 10)
   SW1       <->  P0.01 (Arduino GPIO number 1)
   A0        <->  P0.04/AIN2 (Arduino Analog A2
   A1        <->  P0.31/AIN7 (Arduino Analog A7
   SPI_CS    <->  P0.26 (Arduino GPIO number 26) 
 */
#include <Arduino.h>
#include <LoRaWan-RAK4630.h> //http://librarymanager/All#SX126x
#include <SPI.h>
#include <math.h>
#include "keys.h"

// RAK4630 supply two LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif

#ifndef LED_BUILTIN2
#define LED_BUILTIN2 36
#endif

bool doOTAA = true;
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60										  /**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DATERATE DR_1									  /*LoRaMac datarates definition, from DR_0 to DR_5*/
#define LORAWAN_TX_POWER TX_POWER_5								  /*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
#define JOINREQ_NBTRIALS 3										  /**< Number of trials for the join request. */
DeviceClass_t gCurrentClass = CLASS_A;							  /* class definition*/
lmh_confirm gCurrentConfirm = LMH_CONFIRMED_MSG;				  /* confirm/unconfirm packet definition*/
uint8_t gAppPort = LORAWAN_APP_PORT;							  /* data port*/

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
 */
static lmh_param_t lora_param_init = {LORAWAN_ADR_ON, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);
static uint8_t boardGetBatteryLevel(void);
static void initReadVBAT(void);

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t lora_callbacks = {boardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
										lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler};

//OTAA keys !!!! KEYS ARE MSB !!!!
// NOTE : FILL IN THE THREE REQUIRED HELIUM NETWORK CREDENTIALS WITH YOUR VALUES AND DELETE THIS LINE

uint8_t nodeDeviceEUI[8] = NODE_DEVICE_EUI;
uint8_t nodeAppEUI[8] = NODE_APP_EUI;
uint8_t nodeAppKey[16] = NODE_APP_KEY;

// Private defination
#define LORAWAN_APP_DATA_BUFF_SIZE 64										  /**< buffer size of the data to be transmitted. */
#define LORAWAN_APP_INTERVAL 300000 // not used											  /**< Defines for user timer, the application data transmission interval. 20s, value in [ms]. */
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];			  //< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; //< Lora user application data structure.

TimerEvent_t appTimer;
static uint32_t timers_init(void);
static uint32_t count = 0;
static uint32_t count_fail = 0;

// Define constants
//   _____ _   _ _______ ______ _______      __     _      
//  |_   _| \ | |__   __|  ____|  __ \ \    / /\   | |     
//    | | |  \| |  | |  | |__  | |__) \ \  / /  \  | |     
//    | | | . ` |  | |  |  __| |  _  / \ \/ / /\ \ | |     
//   _| |_| |\  |  | |  | |____| | \ \  \  / ____ \| |____ 
//  |_____|_| \_|  |_|  |______|_|  \_\  \/_/    \_\______|

#define SEND_INTERVAL 5 // minutes 

// Variables for wind data
static double dir_sum_sin = 0;
static double dir_sum_cos = 0;
static float velSum = 0;
static float gust = 0;
static float lull = -1;
static int velCount = 0;
static int dirCount = 0;

// Variables for other metrics
static float batVoltageF = 0;
static float capVoltageF = 0;
static float temperatureF = 0;
static float rain = 0;
static int rainSum = 0;

static bool initialSendDone = false;
// Timer for sending data
unsigned long lastSendTime = 0;
unsigned long send_interval_ms = SEND_INTERVAL * 60000;

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	// Initialize LoRa chip.
	lora_rak4630_init();
     delay(5000);
	// Initialize Serial for debug output
	Serial.begin(115200);
	Serial1.begin(115200);

	// Serial2.setPins(15, 16); // Set RX and TX pins
	// Serial2.begin(115200);
	// this next will cause the app to pause until
	// you connect the virtual terminal to the comm port
	// while (!Serial)
	// {
	// 	delay(10);
	// }
	Serial.println("=====================================");
	Serial.println("Welcome to RAK4630 LoRaWan!!!");
	Serial.println("Type: OTAA");

#ifdef SX126x_Arduino_Ver1

#if defined(REGION_AS923)
	Serial.println("Region: AS923");
#elif defined(REGION_AU915)
	Serial.println("Region: AU915");
#elif defined(REGION_CN470)
	Serial.println("Region: CN470");
#elif defined(REGION_CN779)
	Serial.println("Region: CN779");
#elif defined(REGION_EU433)
	Serial.println("Region: EU433");
#elif defined(REGION_IN865)
	Serial.println("Region: IN865");
#elif defined(REGION_EU868)
	Serial.println("Region: EU868");
#elif defined(REGION_KR920)
	Serial.println("Region: KR920");
#elif defined(REGION_US915)
	Serial.println("Region: US915");
#elif defined(REGION_US915_HYBRID)
	Serial.println("Region: US915_HYBRID");
#else
	Serial.println("Please define a region in the compiler options.");
#endif
	Serial.println("=====================================");

	Serial.println("Built against SX126x-Arduino version 1.x");
#else
	Serial.println("Built against SX126x-Arduino version 2.x");
#endif  // SX126x_Arduino_Ver1

	//creat a user timer to send data to server period
	uint32_t err_code;
	err_code = timers_init();
	if (err_code != 0)
	{
		Serial.printf("timers_init failed - %d\n", err_code);
	}

	// Setup the EUIs and Keys
	lmh_setDevEui(nodeDeviceEUI);
	lmh_setAppEui(nodeAppEUI);
	lmh_setAppKey(nodeAppKey);

	// Initialize LoRaWan
#ifdef SX126x_Arduino_Ver1
	// SX126x-Arduino version 1 API
	err_code = lmh_init(&lora_callbacks, lora_param_init, doOTAA);
#else
	// SX126x-Arduino version 2.x API
	err_code = lmh_init(&lora_callbacks, lora_param_init, doOTAA, CLASS_A, LORAMAC_REGION_US915);
#endif
	if (err_code != 0)
	{
		Serial.printf("lmh_init failed - %d\n", err_code);
	}

	// Start Join procedure
	Serial.println("Starting Join");
	lmh_join();
	Serial.println("Return from join");
	send_interval_ms = 30000; // Start with 30 second interval
	Serial.println("Initial send interval: 30 seconds");
	Serial.println("Will revert to normal interval after first send");
	initReadVBAT();
}

void loop() {
    const int maxIterations = 100; // Maximum number of lines to read per loop
    int iterationCount = 0;

    // Check if data is available on the serial port
    while (Serial1.available() > 0 && iterationCount < maxIterations) {
        iterationCount++;
        String line = Serial1.readStringUntil('\n');
        line.trim();
        #ifdef PRINT_WX_SERIAL
		Serial.println(line);
		#endif
		if (line.length() > 0)
        {
            // Find the '=' character
            int index = line.indexOf('=');
            if (index != -1)
            {
                // Split into key and value, removing whitespace
                String key = line.substring(0, index);
                String value = line.substring(index + 1);
                key.trim();
                value.trim();

                // Remove 'V' suffix from voltage readings if present
                if (value.endsWith("V")) {
                    value = value.substring(0, value.length() - 1);
                }

                // Now parse based on the key
                if (key == "WindDir")
                {
                    float windDir = value.toFloat();
                    double radians = windDir * M_PI / 180.0;
                    dir_sum_sin += sin(radians);
                    dir_sum_cos += cos(radians);
                    dirCount++;
                }
                else if (key == "WindSpeed")
                {
                    float windSpeed = value.toFloat();
                    velSum += windSpeed;
                    velCount++;
                    if (lull == -1 || windSpeed < lull)
                        lull = windSpeed;
                }
                else if (key == "WindGust")
                {
                    float windGust = value.toFloat();
                    if (windGust > gust)
                        gust = windGust;
                }
                else if (key == "BatVoltage")
                {
                    batVoltageF = value.toFloat();
                }
                else if (key == "CapVoltage")
                {
                    capVoltageF = value.toFloat();
                }
                else if (key == "GXTS04Temp" || key == "Temperature")  // Handle both sensor types
                {
                    if (value != "--")  // Check for valid temperature
                    {
                        temperatureF = value.toFloat();
                    }
                }
                // ... rest of your parsing code ...
            }
        }
    }

    if (iterationCount >= maxIterations) {
        Serial.println("Maximum serial reading iterations reached");
    }

    // Check if it's time to send data
    if (millis() - lastSendTime >= send_interval_ms  )
    {
		lastSendTime = millis();

		// After first send, switch to normal interval
		if (!initialSendDone)
		{
			send_interval_ms = SEND_INTERVAL * 60000;
			initialSendDone = true;
			Serial.printf("Switching to normal send interval: %lu minutes\n", send_interval_ms / 60000);
		}
		// Calculate averages
        float velAvg = (velCount > 0) ? velSum / velCount : 0;
        double avgSin = (dirCount > 0) ? dir_sum_sin / dirCount : 0;
        double avgCos = (dirCount > 0) ? dir_sum_cos / dirCount : 0;
        double avgRadians = atan2(avgSin, avgCos);
        float dirAvg = avgRadians * 180.0 / M_PI; // Convert to degrees
        if (dirAvg < 0)
            dirAvg += 360.0;

        // Print data
        Serial.printf("Wind Speed Avg: %.1f m/s, Wind Dir Avg: %d°, Gust: %.1f m/s, Lull: %.1f m/s\n",
                      velAvg, (int)dirAvg, gust, lull);
        Serial.printf("Battery Voltage: %.1f V, Capacitor Voltage: %.1f V, Temperature: %.1f °C\n",
                      batVoltageF, capVoltageF, temperatureF);
        Serial.printf("Rain: %.1f mm, Rain Sum: %d\n", rain, rainSum);

        // Send data over LoRa
        if (lmh_join_status_get() == LMH_SET)
        {

			// Clear the buffer
			memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
			// // Format the data into the buffer
			// int formattedLength = snprintf((char *)m_lora_app_data.buffer, LORAWAN_APP_DATA_BUFF_SIZE,
			// 							   "%.2f,%.2f,%.2f",
			// 							   velAvg, dirAvg, gust);
			// Round the values to one decimal place
			float roundedVelAvg = round(velAvg * 10) / 10.0;
			float roundedDirAvg = round(dirAvg);
			float roundedGust = round(gust * 10) / 10.0;
			float roundedLull = round(lull * 10) / 10.0;
			float roundedBatVoltageF = round(batVoltageF * 10) / 10.0;
			float roundedCapVoltageF = round(capVoltageF * 10) / 10.0;
			float roundedTemperatureF = round(temperatureF * 10) / 10.0;
			float roundedRain = round(rain * 10) / 10.0;

			// Format the data into the buffer
			// int formattedLength = snprintf((char *)m_lora_app_data.buffer, LORAWAN_APP_DATA_BUFF_SIZE,
			// 							   "%.1f,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%d",
			// 							   roundedVelAvg, roundedDirAvg, roundedGust, roundedLull,
			// 							   roundedBatVoltageF, roundedCapVoltageF, roundedTemperatureF, (int)roundedRain);
			// Check if the formatted string fits within the buffer
			// if (formattedLength < 0 || formattedLength >= LORAWAN_APP_DATA_BUFF_SIZE)
			// {
			// 	Serial.println("Error: Formatted string is too large for the buffer.");
			// 	return;
			// }

			// Convert values to integers
			int16_t intVelAvg = (int16_t)(roundedVelAvg * 10);			   // Scale to 1 decimal place
			int16_t intDirAvg = (int16_t)(roundedDirAvg * 10);			   // Scale to 1 decimal place
			int16_t intGust = (int16_t)(roundedGust * 10);				   // Scale to 1 decimal place
			int16_t intLull = (int16_t)(roundedLull * 10);				   // Scale to 1 decimal place
			int16_t intBatVoltageF = (int16_t)(roundedBatVoltageF * 100);  // Scale to 2 decimal places
			int16_t intCapVoltageF = (int16_t)(roundedCapVoltageF * 100);  // Scale to 2 decimal places
			int16_t intTemperatureF = (int16_t)(roundedTemperatureF * 10); // Scale to 1 decimal place
			uint16_t intRain = (uint16_t)(roundedRain * 10);			   // Scale to 1 decimal place
			uint16_t deviceVoltage_mv = (uint16_t)(analogRead(BATTERY_PIN) * REAL_VBAT_MV_PER_LSB);
			// Pack the integers into the buffer in a specific order
			int offset = 0;
			memcpy(&m_lora_app_data.buffer[offset], &intDirAvg, sizeof(int16_t));
			offset += sizeof(int16_t);
			memcpy(&m_lora_app_data.buffer[offset], &intVelAvg, sizeof(int16_t));
			offset += sizeof(int16_t);
			memcpy(&m_lora_app_data.buffer[offset], &intGust, sizeof(int16_t));
			offset += sizeof(int16_t);
			memcpy(&m_lora_app_data.buffer[offset], &intLull, sizeof(int16_t));
			offset += sizeof(int16_t);
			memcpy(&m_lora_app_data.buffer[offset], &intBatVoltageF, sizeof(int16_t));
			offset += sizeof(int16_t);
			memcpy(&m_lora_app_data.buffer[offset], &intCapVoltageF, sizeof(int16_t));
			offset += sizeof(int16_t);
			memcpy(&m_lora_app_data.buffer[offset], &intTemperatureF, sizeof(int16_t));
			offset += sizeof(int16_t);
			memcpy(&m_lora_app_data.buffer[offset], &intRain, sizeof(uint16_t));
			offset += sizeof(uint16_t);
			memcpy(&m_lora_app_data.buffer[offset], &deviceVoltage_mv, sizeof(uint16_t));
			offset += sizeof(uint16_t);

			// Print debug information
			Serial.print("Payload bytes: ");
			for (int i = 0; i < m_lora_app_data.buffsize; i++)
			{
				Serial.printf("%02X", m_lora_app_data.buffer[i]);
			}
			Serial.println();

			// Set the buffer size to the total number of bytes
			m_lora_app_data.buffsize = offset;
			m_lora_app_data.port = gAppPort;

			// // String payload
			// // Set the buffer size to the length of the formatted string
			// m_lora_app_data.buffsize = strlen((char *)m_lora_app_data.buffer);
			// m_lora_app_data.port = gAppPort;
			// Serial.printf("Formatted payload: %s\n", (char *)m_lora_app_data.buffer);
			// Serial.printf("message length : %d\n", m_lora_app_data.buffsize);

			lmh_error_status error;
			int retryCount = 0;
			const int maxRetries = 5;
			do
			{
				error = lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
				if (error == LMH_SUCCESS)
				{
					Serial.println("LoRa data sent successfully.");
					break; // Exit the loop if the send is successful
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
			}
		}
        else
        {
            Serial.println("Not joined to the network. Cannot send data.");
			delay(5000);
			static int not_joined_error_count = 0 ;
			not_joined_error_count++ ;
			if (not_joined_error_count > 5) {
				// reboot.
				NVIC_SystemReset(); // Perform a system reset
			}
			return; // don't reset the counters just yet.
        }

		// Reset counters
		dir_sum_sin = dir_sum_cos = 0; // Reset wind direction sums
		velSum = 0;					   // Reset wind speed sum
		velCount = dirCount = 0;	   // Reset wind direction and speed counts
		gust = 0;					   // Reset gust
		lull = -1;					   // Reset lull

		// Reset other metrics
		batVoltageF = 0;  // Reset battery voltage
		capVoltageF = 0;  // Reset capacitor voltage
		temperatureF = 0; // Reset temperature
		rain = 0;		  // Reset rain
		rainSum = 0;	  // Reset rain sum
	}
}
void initReadVBAT(void)
{
	// Set the analog reference to 3.0V (default = 3.6V)
	analogReference(AR_INTERNAL_3_0);

	// Set the resolution to 12-bit (0..4095)
	analogReadResolution(12); // Can be 8, 10, 12 or 14

	// Let the ADC settle
	delay(1);

	// Get a single ADC sample and throw it away
	boardGetBatteryLevel();
}
uint8_t boardGetBatteryLevel(void) {
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

/**@brief LoRa function for handling HasJoined event.
 */
void lorawan_has_joined_handler(void)
{
	Serial.println("OTAA Mode, Network Joined!");

	lmh_error_status ret = lmh_class_request(gCurrentClass);
	if (ret == LMH_SUCCESS)
	{
		delay(1000);
		TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
		TimerStart(&appTimer);
	}
}

/**@brief Function for handling LoRaWan received data from Gateway
 *
 * @param[in] app_data  Pointer to rx data
 */
void lorawan_rx_handler(lmh_app_data_t *app_data)
{
	Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d, data:%s\n",
				  app_data->port, app_data->buffsize, app_data->rssi, app_data->snr, app_data->buffer);
	// Check if the buffer contains a valid integer for the send interval
	if (app_data->buffsize > 0 && isdigit(app_data->buffer[0]))
	{
		int new_interval = atoi((const char *)app_data->buffer); // Convert buffer to integer
		if (new_interval > 0)
		{
			Serial.printf("Setting send interval to %d minutes\n", new_interval);
			send_interval_ms = new_interval * 60000; // Convert minutes to milliseconds
		}
	}
	// Check if the buffer contains the "reboot" command
	else if (String("reboot").equals(String((const char *)app_data->buffer)))
	{
		Serial.println("Got reboot command");
		NVIC_SystemReset(); // Perform a system reset
	}
}

void lorawan_confirm_class_handler(DeviceClass_t Class)
{
	Serial.printf("switch to class %c done\n", "ABC"[Class]);
	// Informs the server that switch has occurred ASAP
	m_lora_app_data.buffsize = 0;
	m_lora_app_data.port = gAppPort;
	lmh_send(&m_lora_app_data, gCurrentConfirm);
}

void send_lora_frame(void)
{
	return;
	if (lmh_join_status_get() != LMH_SET)
	{
		//Not joined, try again later
		return;
	}

	const char *message = "45 NE 25g30";
	memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE); // Clear buffer
	m_lora_app_data.port = gAppPort;

	// Copy the string into the buffer
	strcpy((char *)m_lora_app_data.buffer, message);

	// Set the buffer size to the length of the string
	m_lora_app_data.buffsize = strlen(message);
	lmh_error_status error = lmh_send(&m_lora_app_data, gCurrentConfirm);
	if (error == LMH_SUCCESS)
	{
		count++;
		Serial.printf("lmh_send ok count %d\n", count);
	}
	else
	{
		count_fail++;
		Serial.printf("lmh_send fail count %d\n", count_fail);
	}
}

/**@brief Function for handling user timerout event.
 */
void tx_lora_periodic_handler(void)
{
	TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
	TimerStart(&appTimer);
	// Serial.println("Sending frame now...");
	// send_lora_frame();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
uint32_t timers_init(void)
{
	TimerInit(&appTimer, tx_lora_periodic_handler);
	return 0;
}

#include "ws8x.h"
#include <Arduino.h>
#include <math.h>

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

void ws8x_init()
{
    // Initialize serial or anything related to ws8x
    Serial1.begin(115200);
}

void ws8x_checkSerial()
{
    const int maxIterations = 100; // Maximum number of lines to read per loop
    int iterationCount = 0;

    // Check if data is available on the serial port
    while (Serial1.available() > 0 && iterationCount < maxIterations)
    {
        iterationCount++;
        String line = Serial1.readStringUntil('\n');
        line.trim();
#ifdef PRINT_WX_SERIAL
        Serial.println(line);
#else
        // Serial.print('.');
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
                if (value.endsWith("V"))
                {
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
                else if (key == "GXTS04Temp" || key == "Temperature") // Handle both sensor types
                {
                    if (value != "--") // Check for valid temperature
                    {
                        temperatureF = value.toFloat();
                    }
                }
                // ... rest of your parsing code ...
            }
        }
    }

    if (iterationCount >= maxIterations)
    {
        Serial.println("Maximum serial reading iterations reached");
    }
}
void ws8x_populate_lora_buffer(lmh_app_data_t *m_lora_app_data, int size)
{
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

    // Populate the buffer

    // Clear the buffer
    memset(m_lora_app_data->buffer, 0, size);

    // Round the values to one decimal place
    float roundedVelAvg = round(velAvg * 10) / 10.0;
    float roundedDirAvg = round(dirAvg);
    float roundedGust = round(gust * 10) / 10.0;
    float roundedLull = round(lull * 10) / 10.0;
    float roundedBatVoltageF = round(batVoltageF * 10) / 10.0;
    float roundedCapVoltageF = round(capVoltageF * 10) / 10.0;
    float roundedTemperatureF = round(temperatureF * 10) / 10.0;
    float roundedRain = round(rain * 10) / 10.0;

    // Convert values to integers
    int16_t intVelAvg = (int16_t)(roundedVelAvg * 10);             // Scale to 1 decimal place
    int16_t intDirAvg = (int16_t)(roundedDirAvg * 10);             // Scale to 1 decimal place
    int16_t intGust = (int16_t)(roundedGust * 10);                 // Scale to 1 decimal place
    int16_t intLull = (int16_t)(roundedLull * 10);                 // Scale to 1 decimal place
    int16_t intBatVoltageF = (int16_t)(roundedBatVoltageF * 100);  // Scale to 2 decimal places
    int16_t intCapVoltageF = (int16_t)(roundedCapVoltageF * 100);  // Scale to 2 decimal places
    int16_t intTemperatureF = (int16_t)(roundedTemperatureF * 10); // Scale to 1 decimal place
    uint16_t intRain = (uint16_t)(roundedRain * 10);               // Scale to 1 decimal place
    uint16_t deviceVoltage_mv = (uint16_t)(analogRead(BATTERY_PIN) * REAL_VBAT_MV_PER_LSB);
    // Pack the integers into the buffer in a specific order
    int offset = 0;
    memcpy(&m_lora_app_data->buffer[offset], &intDirAvg, sizeof(int16_t));
    offset += sizeof(int16_t);
    memcpy(&m_lora_app_data->buffer[offset], &intVelAvg, sizeof(int16_t));
    offset += sizeof(int16_t);
    memcpy(&m_lora_app_data->buffer[offset], &intGust, sizeof(int16_t));
    offset += sizeof(int16_t);
    memcpy(&m_lora_app_data->buffer[offset], &intLull, sizeof(int16_t));
    offset += sizeof(int16_t);
    memcpy(&m_lora_app_data->buffer[offset], &intBatVoltageF, sizeof(int16_t));
    offset += sizeof(int16_t);
    memcpy(&m_lora_app_data->buffer[offset], &intCapVoltageF, sizeof(int16_t));
    offset += sizeof(int16_t);
    memcpy(&m_lora_app_data->buffer[offset], &intTemperatureF, sizeof(int16_t));
    offset += sizeof(int16_t);
    memcpy(&m_lora_app_data->buffer[offset], &intRain, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    memcpy(&m_lora_app_data->buffer[offset], &deviceVoltage_mv, sizeof(uint16_t));
    offset += sizeof(uint16_t);

    // Print debug information
    Serial.print("Payload bytes: ");
    for (int i = 0; i < m_lora_app_data->buffsize; i++)
    {
        Serial.printf("%02X", m_lora_app_data->buffer[i]);
    }
    Serial.println();

    // Set the buffer size to the total number of bytes
    m_lora_app_data->buffsize = offset;
}

void ws8x_reset_counters() {
    // Reset counters
    dir_sum_sin = dir_sum_cos = 0; // Reset wind direction sums
    velSum = 0;                    // Reset wind speed sum
    velCount = dirCount = 0;       // Reset wind direction and speed counts
    gust = 0;                      // Reset gust
    lull = -1;                     // Reset lull

    // Reset other metrics
    batVoltageF = 0;  // Reset battery voltage
    capVoltageF = 0;  // Reset capacitor voltage
    temperatureF = 0; // Reset temperature
    rain = 0;         // Reset rain
    rainSum = 0;      // Reset rain sum
}
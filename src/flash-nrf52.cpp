/**
 * @file flash-nrf52.cpp
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief Initialize, read and write parameters from/to internal flash memory
 * @version 0.1
 * @date 2021-01-10
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "main.h"

/** Flag if data flash was initialized */
bool init_flash_done;

/** Buffer for flash content */
s_lorawan_settings g_flash_content;

#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;

/** Name for settings file */
const char settings_name[] = "RAK";

/** File instance */
File lora_file(InternalFS);

/**
 * @brief Initialize access to nRF52 internal file system
 *
 */
void init_flash(void)
{
	if (init_flash_done)
	{
		return;
	}

	// Initialize Internal File System
	InternalFS.begin();

	// Check if file exists
	lora_file.open(settings_name, FILE_O_READ);
	if (!lora_file)
	{
		APP_LOG("FLASH", "File doesn't exist, force format");
		delay(1000);
		flash_reset();
		lora_file.open(settings_name, FILE_O_READ);
	}

	uint8_t markers[2] = {0};
	lora_file.read(markers, 2);
	// Found new structure
	lora_file.close();
	lora_file.open(settings_name, FILE_O_READ);
	lora_file.read((uint8_t *)&g_lorawan_settings, sizeof(s_lorawan_settings));
	lora_file.close();
	// Check if it is LPWAN settings
	if ((g_lorawan_settings.valid_mark_1 != 0xAA) || (g_lorawan_settings.valid_mark_2 != LORAWAN_DATA_MARKER))
	{
		// Data is not valid, reset to defaults
		APP_LOG("FLASH", "Invalid data set, deleting and restart node");
		InternalFS.format();
		delay(1000);
		sd_nvic_SystemReset();
	}
	init_flash_done = true;
}

/**
 * @brief Save changed settings if required
 *
 * @return boolean
 * 			result of saving
 */
boolean save_settings(void)
{
	bool result = true;
	// Read saved content
	lora_file.open(settings_name, FILE_O_READ);
	if (!lora_file)
	{
		APP_LOG("FLASH", "File doesn't exist, force format");
		delay(100);
		flash_reset();
		lora_file.open(settings_name, FILE_O_READ);
	}
	lora_file.read((uint8_t *)&g_flash_content, sizeof(s_lorawan_settings));
	lora_file.close();
	if (memcmp((void *)&g_flash_content, (void *)&g_lorawan_settings, sizeof(s_lorawan_settings)) != 0)
	{
		APP_LOG("FLASH", "Flash content changed, writing new data");
		delay(100);

		InternalFS.remove(settings_name);

		if (lora_file.open(settings_name, FILE_O_WRITE))
		{
			lora_file.write((uint8_t *)&g_lorawan_settings, sizeof(s_lorawan_settings));
			lora_file.flush();
		}
		else
		{
			result = false;
		}
		lora_file.close();
	}
	return result;
}

/**
 * @brief Reset content of the filesystem
 *
 */
void flash_reset(void)
{
	InternalFS.format();
	if (lora_file.open(settings_name, FILE_O_WRITE))
	{
		s_lorawan_settings default_settings;
		lora_file.write((uint8_t *)&default_settings, sizeof(s_lorawan_settings));
		lora_file.flush();
		lora_file.close();
	}
}
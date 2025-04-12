/**
 * @file main.h
 * @author Bernd Giesecke (bernd@giesecke.tk)
 * @brief Defines and includes
 * @version 0.1
 * @date 2025-04-02
 *
 * @copyright Copyright (c) 2025
 *
 */
#include <Arduino.h>
#include <nrf_nvic.h>
#include <LoRaWan-Arduino.h>
#include "keys.h"

// Debug output set to 0 to disable app debug output
#ifndef APP_DEBUG
#define APP_DEBUG 0
#endif

#if APP_DEBUG > 0
#define APP_LOG(tag, ...)                   \
	do                                      \
	{                                       \
		if (tag)                            \
			PRINTF("[%s] ", tag);           \
		PRINTF(__VA_ARGS__);                \
		PRINTF("\r\n");                     \
		if (g_ble_uart_is_connected)        \
		{                                   \
			g_ble_uart.printf(__VA_ARGS__); \
			g_ble_uart.printf("\r\n");      \
			g_ble_uart.flush();             \
		}                                   \
	} while (0)
#else
#define APP_LOG(...)
#endif

/** Wake up events, more events can be defined in app.h */
#define NO_EVENT 0
#define BLE_CONFIG 0b0000000000000010
#define N_BLE_CONFIG 0b1111111111111101
#define BLE_DATA 0b0000000000000100
#define N_BLE_DATA 0b1111111111111011
#define AT_CMD 0b0000000000100000
#define N_AT_CMD 0b1111111111011111
extern volatile uint16_t g_task_event_type;

// BLE
#include <bluefruit.h>
void init_ble(void);
void restart_advertising(uint16_t timeout);
extern BLECharacteristic g_lora_data;
extern BLEUart g_ble_uart;
extern bool g_ble_uart_is_connected;
extern char g_ble_dev_name[];

// LoRaWAN/LoRa P2P
int8_t init_lora(void);
int8_t init_lorawan(bool region_change = false);
bool send_p2p_packet(uint8_t *data, uint8_t size);
lmh_error_status send_lora_packet(uint8_t *data, uint8_t size, uint8_t fport = 1);

#define LORAWAN_DATA_MARKER 0x55
struct s_lorawan_settings
{
	uint8_t valid_mark_1 = 0xAA;				// Just a marker for the Flash
	uint8_t valid_mark_2 = LORAWAN_DATA_MARKER; // Just a marker for the Flash
												// OTAA Device EUI MSB
	uint8_t node_device_eui[8] = NODE_DEVICE_EUI;
	// OTAA Application EUI MSB
	uint8_t node_app_eui[8] = NODE_APP_EUI;
	// OTAA Application Key MSB
	uint8_t node_app_key[16] = NODE_APP_KEY;
	// ABP Device Address MSB
	uint32_t node_dev_addr = 0x26021FB4;
	// ABP Network Session Key MSB
	uint8_t node_nws_key[16] = {0x32, 0x3D, 0x15, 0x5A, 0x00, 0x0D, 0xF3, 0x35, 0x30, 0x7A, 0x16, 0xDA, 0x0C, 0x9D, 0xF5, 0x3F};
	// ABP Application Session key MSB
	uint8_t node_apps_key[16] = {0x3F, 0x6A, 0x66, 0x45, 0x9D, 0x5E, 0xDC, 0xA6, 0x3C, 0xBC, 0x46, 0x19, 0xCD, 0x61, 0xA1, 0x1E};
	// Flag for OTAA or ABP
	bool otaa_enabled = true;
	// Flag for ADR on or off
	bool adr_enabled = false;
	// Flag for public or private network
	bool public_network = true;
	// Flag to enable duty cycle
	bool duty_cycle_enabled = false;
	// Default is off
	uint32_t send_repeat_time = 0;
	// Number of join retries
	uint8_t join_trials = 5;
	// TX power 0 .. 10
	uint8_t tx_power = TX_POWER_5;
	// Data rate 0 .. 15 (validity depnends on Region)
	uint8_t data_rate = 2;
	// LoRaWAN class 0: A, 2: C, 1: B is not supported
	uint8_t lora_class = 0;
	// Subband channel selection 1 .. 9
	uint8_t subband_channels = 1;
	// Flag if node joins automatically after reboot
	bool auto_join = true;
	// Data port to send data
	uint8_t app_port = 2;
	// Flag to enable confirmed messages
	lmh_confirm confirmed_msg_enabled = LMH_UNCONFIRMED_MSG;
	// Fixed LoRaWAN lorawan_region (depends on compiler option)
	uint8_t lora_region = 1;
	// Flag for LoRaWAN or LoRa P2P
	bool lorawan_enable = true;
	// Frequency in Hz
	uint32_t p2p_frequency = 916000000;
	// Tx power 0 .. 22
	uint8_t p2p_tx_power = 22;
	// Bandwidth 0: 125, 1: 250, 2: 500, 3: 62.5, 4: 41.67, 5: 31.25, 6: 20.83, 7: 15.63, 8: 10.4, 9: 7.8
	uint8_t p2p_bandwidth = 0;
	// Spreading Factor SF7..SF12
	uint8_t p2p_sf = 7;
	// Coding Rate 1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8
	uint8_t p2p_cr = 1;
	// Preamble length
	uint8_t p2p_preamble_len = 8;
	// Symbol timeout
	uint16_t p2p_symbol_timeout = 0;
	// Command from BLE to reset device
	bool resetRequest = true;
};

extern s_lorawan_settings g_lorawan_settings;
extern bool g_lpwan_has_joined;
extern uint8_t g_rx_lora_data[];
extern uint8_t g_rx_data_len;
extern uint8_t g_tx_lora_data[];
extern uint8_t g_tx_data_len;
extern bool g_lorawan_initialized;
extern int16_t g_last_rssi;
extern int8_t g_last_snr;
extern bool g_rx_fin_result;

enum P2P_RX_MODE
{
	RX_MODE_NONE = 0,
	RX_MODE_RX = 1,
	RX_MODE_RX_TIMED = 2,
	RX_MODE_RX_WAIT = 3
};
extern uint8_t g_lora_p2p_rx_mode;
extern uint32_t g_lora_p2p_rx_time;
extern bool g_rx_continuous;


// Flash
void init_flash(void);
bool save_settings(void);
void flash_reset(void);
extern bool init_flash_done;

// Battery
void init_batt(void);
float read_batt(void);
uint8_t get_lora_batt(void);
uint8_t mv_to_percent(float mvolts);

// The battery sense is hooked to pin A0 (5)
#define BATTERY_PIN A0
/** Definition of milliVolt per LSB => 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096 */
#define VBAT_MV_PER_LSB (0.73242188F)
/** Voltage divider value => 1.5M + 1M voltage divider on VBAT = (1.5M / (1M + 1.5M)) */
#define VBAT_DIVIDER (0.4F)
/** Compensation factor for the VBAT divider */
#define VBAT_DIVIDER_COMP (1.73)
/** Fixed calculation of milliVolt from compensation value */
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

// AT command parser
typedef struct atcmd_s
{
	const char *cmd_name;		   // CMD NAME
	const char *cmd_desc;		   // AT+CMD?
	int (*query_cmd)(void);		   // AT+CMD=?
	int (*exec_cmd)(char *str);	   // AT+CMD=value
	int (*exec_cmd_no_para)(void); // AT+CMD
	const char *permission;		   // "R" or "RW"
} atcmd_t;

void at_serial_input(uint8_t cmd);
extern char *region_names[];
extern char g_at_query_buf[];
extern atcmd_t *g_user_at_cmd_list __attribute__((weak));
extern uint8_t g_user_at_cmd_num __attribute__((weak));
extern bool has_custom_at;

#define AT_PRINTF(...)                  \
	Serial.printf(__VA_ARGS__);         \
	Serial.printf("\r\n");              \
	if (g_ble_uart_is_connected)        \
	{                                   \
		g_ble_uart.printf(__VA_ARGS__); \
		g_ble_uart.printf("\r\n");      \
		g_ble_uart.flush();             \
	}

#define AT_ERROR "+CME ERROR:"
#define ATCMD_SIZE 160
#define ATQUERY_SIZE 512

#define AT_SUCCESS (0)
#define AT_ERRNO_NOSUPP (1)
#define AT_ERRNO_NOALLOW (2)
#define AT_ERRNO_PARA_VAL (5)
#define AT_ERRNO_PARA_NUM (6)
#define AT_ERRNO_EXEC_FAIL (7)
#define AT_ERRNO_SYS (8)
#define AT_CB_PRINT (0xFF)

// Forward declarations
extern uint16_t g_sw_ver_1; // major version increase on API change / not backwards compatible
extern uint16_t g_sw_ver_2; // minor version increase on API change / backward compatible
extern uint16_t g_sw_ver_3; // patch version increase on bugfix, no affect on API
extern uint8_t g_last_fport;



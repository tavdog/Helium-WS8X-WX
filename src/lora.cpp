/**
 * @file lora.cpp
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief LoRaWAN initialization & handler
 * @version 0.1
 * @date 2021-09-15
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "main.h"

// LoRa callbacks
static RadioEvents_t RadioEvents;
void on_tx_done(void);
void on_rx_done(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void on_tx_timeout(void);
void on_rx_timeout(void);
void on_rx_crc_error(void);
void on_cad_done(bool cadResult);

uint8_t g_lora_p2p_rx_mode = RX_MODE_NONE;
uint32_t g_lora_p2p_rx_time = 0;

/**
 * @brief Initialize LoRa HW and LoRaWan MAC layer
 *
 * @return int8_t result
 *  0 => OK
 * -1 => SX126x HW init failure
 */
int8_t init_lora(void)
{
	if (!g_lorawan_initialized)
	{
		// Initialize LoRa chip.
		if (lora_rak4630_init() != 0)
		{
			APP_LOG("LORA", "Failed to initialize SX1262");
			return -1;
		}

		// Initialize the Radio
		RadioEvents.TxDone = on_tx_done;
		RadioEvents.RxDone = on_rx_done;
		RadioEvents.TxTimeout = on_tx_timeout;
		RadioEvents.RxTimeout = on_rx_timeout;
		RadioEvents.RxError = on_rx_crc_error;
		RadioEvents.CadDone = on_cad_done;

		Radio.Init(&RadioEvents);
	}
	Radio.Sleep(); // Radio.Standby();

	Radio.SetChannel(g_lorawan_settings.p2p_frequency);

	Radio.SetTxConfig(MODEM_LORA, g_lorawan_settings.p2p_tx_power, 0, g_lorawan_settings.p2p_bandwidth,
					  g_lorawan_settings.p2p_sf, g_lorawan_settings.p2p_cr,
					  g_lorawan_settings.p2p_preamble_len, false,
					  true, 0, 0, false, 5000);

	Radio.SetRxConfig(MODEM_LORA, g_lorawan_settings.p2p_bandwidth, g_lorawan_settings.p2p_sf,
					  g_lorawan_settings.p2p_cr, 0, g_lorawan_settings.p2p_preamble_len,
					  g_lorawan_settings.p2p_symbol_timeout, false,
					  0, true, 0, 0, false, true);

	switch (g_lora_p2p_rx_mode)
	{
	default:
	case RX_MODE_NONE:
		// No RX mode, do not start RX
		Radio.Sleep();
		break;
	case RX_MODE_RX:
	case RX_MODE_RX_WAIT:
		Radio.Rx(0);
		break;
	case RX_MODE_RX_TIMED:
		Radio.Rx(g_lora_p2p_rx_time);
		break;
	}

	digitalWrite(LED_GREEN, LOW);

	g_lorawan_initialized = true;
	g_lpwan_has_joined = true;

	APP_LOG("LORA", "LoRa initialized");
	return 0;
}

/**
 * @brief Function to be executed on Radio Tx Done event
 */
void on_tx_done(void)
{
	digitalWrite(LED_GREEN, LOW);
	AT_PRINTF("+EVT:TXP2P_DONE")
	g_rx_fin_result = true;

	// Set RX mode
	switch (g_lora_p2p_rx_mode)
	{
	default:
	case RX_MODE_NONE:
		// No RX mode, do not start RX
		Radio.Sleep();
		APP_LOG("LORA", "TX finished - Do not start RX");
		break;
	case RX_MODE_RX:
	case RX_MODE_RX_WAIT:
		Radio.Rx(0);
		APP_LOG("LORA", "TX finished - Start RX");
		break;
	case RX_MODE_RX_TIMED:
		Radio.Rx(g_lora_p2p_rx_time);
		APP_LOG("LORA", "TX finished - Start timed RX");
		break;
	}
}

/**@brief Function to be executed on Radio Rx Done event
 */
void on_rx_done(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
	digitalWrite(LED_GREEN, LOW);

	g_last_rssi = rssi;
	g_last_snr = snr;
	g_rx_fin_result = true;

	// Copy the data into loop data buffer
	memcpy(g_rx_lora_data, payload, size);
	g_rx_data_len = size;

	char log_buff[g_rx_data_len * 2] = {0};
	uint8_t log_idx = 0;
	for (int idx = 0; idx < g_rx_data_len; idx++)
	{
		sprintf(&log_buff[log_idx], "%02X", g_rx_lora_data[idx]);
		log_idx += 3;
	}
	AT_PRINTF("+EVT:RXP2P:%d:%d:%s", g_last_rssi, g_last_snr, log_buff);

		// Set RX mode
	switch (g_lora_p2p_rx_mode)
	{
	default:
	case RX_MODE_NONE:
	case RX_MODE_RX_WAIT:
	case RX_MODE_RX_TIMED:
		// No RX mode, do not start RX
		Radio.Sleep();
		APP_LOG("LORA", "RX finished - Stopping RX");
		break;
	case RX_MODE_RX:
		Radio.Rx(0);
		APP_LOG("LORA", "RX finished - Restarting RX");
		break;
	}
}

/**@brief Function to be executed on Radio Tx Timeout event
 */
void on_tx_timeout(void)
{
	AT_PRINTF("+EVT:TXP2P_TIMEOUT");
	digitalWrite(LED_GREEN, LOW);
	g_rx_fin_result = false;

	// Set RX mode
	switch (g_lora_p2p_rx_mode)
	{
	default:
	case RX_MODE_NONE:
	case RX_MODE_RX_WAIT:
	case RX_MODE_RX_TIMED:
		// No RX mode, do not start RX
		Radio.Sleep();
		APP_LOG("LORA", "TX timeout - Do not start RX");
		break;
	case RX_MODE_RX:
		Radio.Rx(0);
		APP_LOG("LORA", "TX timeout - Restart RX");
		break;
	}
}

/**@brief Function to be executed on Radio Rx Timeout event
 */
void on_rx_timeout(void)
{
	AT_PRINTF("+EVT:RXP2P_TIMEOUT");
	digitalWrite(LED_GREEN, LOW);

	// Set RX mode
	switch (g_lora_p2p_rx_mode)
	{
	default:
	case RX_MODE_NONE:
	case RX_MODE_RX_WAIT:
	case RX_MODE_RX_TIMED:
		// No RX mode, do not start RX
		Radio.Sleep();
		APP_LOG("LORA", "RX timeout - Do not start RX");
		break;
	case RX_MODE_RX:
		Radio.Rx(0);
		APP_LOG("LORA", "RX timeout - Restart RX");
		break;
	}
}

/**@brief Function to be executed on Radio Rx Error event
 */
void on_rx_crc_error(void)
{
	AT_PRINTF("+EVT:TXP2P_CRC_ERROR");
	digitalWrite(LED_GREEN, LOW);

	// Set RX mode
	switch (g_lora_p2p_rx_mode)
	{
	default:
	case RX_MODE_NONE:
	case RX_MODE_RX_WAIT:
	case RX_MODE_RX_TIMED:
		// No RX mode, do not start RX
		Radio.Sleep();
		APP_LOG("LORA", "RX CRC error - Do not start RX");
		break;
	case RX_MODE_RX:
		Radio.Rx(0);
		APP_LOG("LORA", "RX CRC error - Restart RX");
		break;
	}
}

/**@brief Function to be executed on Radio Rx Error event
 */
void on_cad_done(bool cadResult)
{
	if (cadResult)
	{
		digitalWrite(LED_GREEN, LOW);
		// Set RX mode
		switch (g_lora_p2p_rx_mode)
		{
		default:
		case RX_MODE_NONE:
		case RX_MODE_RX_WAIT:
		case RX_MODE_RX_TIMED:
			// No RX mode, do not start RX
			Radio.Sleep();
			APP_LOG("LORA", "CAD failed - Do not start RX");
			break;
		case RX_MODE_RX:
			Radio.Rx(0);
			APP_LOG("LORA", "CAD failed - Restart RX");
			break;
		}

		g_rx_fin_result = false;
	}
	else
	{
		Radio.Send(g_tx_lora_data, g_tx_data_len);
	}
}

/**
 * @brief Prepare packet to be sent and start CAD routine
 *
 */
bool send_p2p_packet(uint8_t *data, uint8_t size)
{
	if (size > 256)
	{
		return false;
	}
	g_tx_data_len = size;
	memcpy(g_tx_lora_data, data, size);

	// Prepare LoRa CAD
	Radio.Sleep();
	Radio.SetCadParams(LORA_CAD_08_SYMBOL, g_lorawan_settings.p2p_sf + 13, 10, LORA_CAD_ONLY, 0);

	// Switch on Indicator lights
	digitalWrite(LED_GREEN, HIGH);

	// Start CAD
	Radio.StartCad();

	return true;
}

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "ws8x.h"
#include "keys.h"

extern bool wakeByUart;
/* OTAA para*/
uint8_t devEui[8] = NODE_DEVICE_EUI;
uint8_t appEui[8] = NODE_APP_EUI;
uint8_t appKey[16] = NODE_APP_KEY;

/* ABP para*/
uint8_t nwkSKey[] = {0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda, 0x85};
uint8_t appSKey[] = {0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef, 0x67};
uint32_t devAddr = (uint32_t)0x007e6ae1;

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = {0xFF00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 30000;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = false; //LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconf
irmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;

uint8_t confirmedNbTrials = 4;

/* Prepares the payload of the frame */
static void prepareTxFrame(uint8_t port)
{
	/*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
	 *appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
	 *if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
	 *if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
	 *for example, if use REGION_CN470,
	 *the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
	 */
	appDataSize = 18;
	appData[0] = 0x00;
	appData[1] = 0x01;
	appData[2] = 0x02;
	appData[3] = 0x03;
}

void setup()
{
	boardInitMcu();
	Serial.begin(115200);
	delay(5000);
	deviceState = DEVICE_STATE_INIT;
	LoRaWAN.ifskipjoin();
		
	wakeByUart = true;
}

void loop()
{
	ws8x_checkSerial();
	// Serial.print(".");
	switch (deviceState)
	{
	case DEVICE_STATE_INIT:
	{
// #if (AT_SUPPORT)
// 		getDevParam();
// #endif
		printDevParam();
		LoRaWAN.init(loraWanClass, loraWanRegion);
		deviceState = DEVICE_STATE_JOIN;
		break;
	}
	case DEVICE_STATE_JOIN:
	{
		LoRaWAN.join();
		break;
	}
	case DEVICE_STATE_SEND:
	{
		ws8x_populate_lora_buffer(appData, appDataSize);
		// prepareTxFrame(appPort);
		LoRaWAN.send();
		deviceState = DEVICE_STATE_CYCLE;
		break;
	}
	case DEVICE_STATE_CYCLE:
	{
		// Schedule next packet transmission
		txDutyCycleTime = appTxDutyCycle + randr(0, APP_TX_DUTYCYCLE_RND);
		LoRaWAN.cycle(txDutyCycleTime);
		deviceState = DEVICE_STATE_SLEEP;
		break;
	}
	case DEVICE_STATE_SLEEP:
	{

		LoRaWAN.sleep();
		break;
	}
	default:
	{
		deviceState = DEVICE_STATE_INIT;
		break;
	}
	}

}

void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
    Serial.printf("Received downlink: %d bytes\n", mcpsIndication->BufferSize);
    
    // Print received data in hex
    Serial.print("Hex: ");
    for(uint8_t i = 0; i < mcpsIndication->BufferSize; i++) {
        Serial.printf("%02X ", mcpsIndication->Buffer[i]);
    }
    Serial.println();
    
    // Print received data in ASCII
    Serial.print("ASCII: ");
    for(uint8_t i = 0; i < mcpsIndication->BufferSize; i++) {
        Serial.print((char)mcpsIndication->Buffer[i]);
    }
    Serial.println();
    
    // Check for "reboot" command (6 bytes)
    if(mcpsIndication->BufferSize == 6) {
        char cmd[7];  // +1 for null terminator
        memcpy(cmd, mcpsIndication->Buffer, 6);
        cmd[6] = '\0';
        
        if(strcmp(cmd, "reboot") == 0) {
            Serial.println("Rebooting...");
            delay(1000);  // Give some time for the message to be sent
            NVIC_SystemReset();
            return;
        }
    }
    
    // Handle duty cycle changes
    if(mcpsIndication->BufferSize == 1) {
        uint8_t value = mcpsIndication->Buffer[0];
        
        // Check if it's an ASCII number (0x30-0x39)
        if(value >= '0' && value <= '9') {
            uint8_t minutes = value - '0';  // Convert ASCII to number
            if(minutes >= 1) {  // No need to check upper bound since single digit
                appTxDutyCycle = (uint32_t)minutes * 60000; // Convert minutes to milliseconds
                Serial.printf("New duty cycle set to: %d minutes (%lu ms)\n", minutes, appTxDutyCycle);
            }
        }
    }
}

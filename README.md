# RAK4631-BLE-LORA-Setup
Example for RAK4631 LoRa/LoRaWAN setup over BLE and with AT commands     
Written for PlatformIO, but can be used in Arduino IDE by renaming **`main.cpp`** to **`src.ino`** and open it with Arduino IDE    

Required library is only beegee-tokyo/SX126x-Arduino    

Kept as simple as possible, but to setup the device it requires
- save and read settings from flash ==> [flash-nrf52.cpp](./flash-nrf52.cpp)
- enable BLE advertising and initialize BLE characteristics for BLE UART and custom settings ==> [ble-nrf52.cpp](./ble-nrf52.cpp)
- AT command parser ==> [at_cmd.cpp](./at_cmd.cpp)
- LoRa and LoRaWAN basic functionality, as the AT command parser needs information from LoRa and LoRaWAN driver [lora.cpp](./lora.cpp) & [lorawan.cpp](./lorawan.cpp)

## Important #1
The AT commands are compatible with the [RUI3 AT commands](https://docs.rakwireless.com/product-categories/software-apis-and-libraries/rui3/at-command-manual).      
But only essential commands are implemented here.

Available commands can be queried over USB or BLE UART with **`AT?`**    

AT commands _**MUST**_ be ending with **`/r/n`**.     

## Optional
In addition to AT commands over USB or BLE UART, the settings can be setup with the [WisBlock Tool Box](https://play.google.com/store/apps/details?id=tk.giesecke.wisblock_toolbox). But WisBlock Tool Box might not be supported by the latest Android versions.

## Important #2
The different parts need to be initialized in **`setup()`**
```cpp
	// Get LoRa parameter
	init_flash();

	// Enable BLE
	APP_LOG("SETUP", "Init BLE");

	// Init BLE
	init_ble();
```

It is recommended to keep [flash-nrf52.cpp](./flash-nrf52.cpp), [ble-nrf52.cpp](./ble-nrf52.cpp), [at_cmd.cpp](./at_cmd.cpp), [lora.cpp](./lora.cpp) and [lorawan.cpp](./lorawan.cpp) unchanged and build your code around this example.    

## Important #3
BLE is kept advertising all the time for testing purposes. To change the advertising time after restart, change
```cpp
	// Keep BLE advertising forever
	restart_advertising(0);
```
in **`setup()`** by changing the **`0`** to an interval in seconds that you want the BLE to be active before it shuts down for power savings.

A more detailed README will be written when I find the time for it.

## Important #4
_**This was put together from different applications I wrote, mainly from the [WisBlock-API-V2](https://github.com/beegee-tokyo/WisBlock-API-V2) and is not complete tested. Use it on your own risk!**_
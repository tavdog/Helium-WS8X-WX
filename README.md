
# Helium WS85 WX

<img width="834" alt="Screenshot 2025-03-19 at 4 53 52 PM" src="https://github.com/user-attachments/assets/ae234304-c6d3-4232-9f90-df9a4d58c139" />

## Overview  
This repository contains a **basic example** of a **WisBlock/Helium** embedded project using **PlatformIO** as the development environment instead of the Arduino IDE.  

This project joins the **Helium network** and periodically transmits **weather (WX) data** from a **WS85 serial output** to the **Helium network**.  

The build setup is based on https://docs.helium.com/network-iot/devices/development/rakwireless/wisblock-4631/platformio
and the code is based on https://github.com/helium/longfi-platformio/tree/master/RAKWireless-WisBlock/examples/LoRaWan_OTAA

### Hardware
The only hardware required is:
* the [WisBlock Starter Kit](https://store.rakwireless.com/products/wisblock-starter-kit) containing  the base board with the core module installed.
* one USB 2.0 A-Male to Micro B interface cable.
* WS85 Weather station from Ecowitt (follow this hackaday article for how to modify the station to enable serial output [WS85 Mod](https://hackaday.io/project/196990-meshtastic-ultrasonic-anemometer-wx-station) )

#### Antenna Type/location
The WisBlock starter kit comes with two antenna types, 
* the one that resembles an "I" is the LoRa antenna, this one connects to the connector on the core moduke marked LoRa, which is below the large K in the RAK logo.
* the one that resembles a "T" is the BLE antenna, this one connects to the connector on the core module marked BLE



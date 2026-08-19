# ESP32C3-Energy
Home Energy Monitoring System

![Screenshot](https://github.com/Ingineerix/ESP32C3-Energy/blob/c7aaafaa1f09b8c38d7a37cc88d2482767cd87e9/ESP32C3-Energy.jpg?raw=true)


Designed for use in any ESP32-C3 equipped Energy Monitor that uses the BL0906 6-Channel Energy Monitor IC.   I used the "IoTorero 6 CH Energy Meter".  It's available on Aliexpress for about $80.   Be sure to get the number of CTs (Current Transformers) you need for the number of inputs you are using.  If you are monitoring split-phase 240VAC, you need 2, but if it's an appliance that doesn't use neutral, you can use one.  (As I did for my EV charger)  

Uses Arduino ESP32 Framework.

Edit config.h with your specifics.

Must Partition with SPIFFS!

Adds Telnet console log access in parallel with UART.

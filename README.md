# RFM95-LoraShield_Arduino for Node with RFM95

My modified code to support RFM95, working with my Single Channel Packet forwarder, tuned at 920.0MHz, SF10, DR0 

Download .zip LMIC-Arduino library from https://github.com/matthijskooijman/arduino-lmic

The list of the files here except .ino should be pasted into the arduino-lmic-master library. Paste under "src/lmic" folder.

The essential change to make it work on my AUS-like frequencies is lorabase.h and lmic.c .
config.h changes also needed as mentioned by the original author's readme file.

For Distance Sensor (SR-04), download library from here : https://github.com/PaulStoffregen/NewPing


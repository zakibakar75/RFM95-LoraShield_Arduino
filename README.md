# RFM95-LoraShield_Arduino for Node with RFM95
My modified code to support RFM95, working with my set of freqs

The list of the files here expect .ino should be pasted into the arduino-lmic-master library.  Follow the original path of the library.
Oh ya, u got to add the library first.  Just Google.

The essential change to make it work on my AUS-like frequencies is lorabase.h and lmic.c .
config.h changes also needed as mentioned by the original author's readme file.

#!/bin/bash

# This script can be used to download a combined bootloader+firmware image
# onto the SAMD20 chip using OpenOCD.

TARGET=/home/jjwsei/Embedded/mamaRoo-Atmel/atmel/build/release/mamaRoo.bin
LOAD_ADDRESS=0x0

OPENOCD_SERVER_PORT=4444

# Make sure openOCD server is running before launching this script.
# Make sure target binary exists before runnning this script.
# See openocd manual for explanation of commands below

# Flash image onto target
echo halt | nc localhost ${OPENOCD_SERVER_PORT} 
echo "Writing image, this might take a few seconds..."
echo flash write_image erase ${TARGET} ${LOAD_ADDRESS} | nc localhost ${OPENOCD_SERVER_PORT} 
echo "Setup internal EEPROM emulation area..."
echo at91samd eeprom 4096 | nc localhost ${OPENOCD_SERVER_PORT} 
# Verify image
echo verify_image ${TARGET} ${LOAD_ADDRESS} | nc localhost ${OPENOCD_SERVER_PORT}
echo reset | nc localhost ${OPENOCD_SERVER_PORT} 
echo halt | nc localhost ${OPENOCD_SERVER_PORT} 
echo at91samd eeprom | nc localhost ${OPENOCD_SERVER_PORT} 

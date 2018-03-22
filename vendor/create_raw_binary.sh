#!/bin/bash

TARGET=build/release/GccGenD20

# Generate raw binary, dissassembly, and print size of executable
arm-none-eabi-objcopy -O binary ${TARGET}.elf ${TARGET}.bin
arm-none-eabi-objcopy -O ihex -R .eeprom -R .fuse -R .lock -R .signature ${TARGET}.elf ${TARGET}.hex
arm-none-eabi-objdump -h -S ${TARGET}.elf > ${TARGET}.lss

# Print section info
#readelf -S ${TARGET}.elf

# Print flash and RAM memory usage info
echo ""
arm-none-eabi-size ${TARGET}.elf
echo "Flash Usage (bytes) = text + data sections"
echo "RAM Usage   (bytes  = data + bss sections"

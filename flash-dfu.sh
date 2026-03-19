#!/bin/bash
# Flash STM32F722 via DFU (BOOT0 high on power-up)
set -e

echo "Building..."
cargo build-stm

ELF_PATH="target/thumbv7em-none-eabihf/release/kasarisw"
BIN_PATH="target/thumbv7em-none-eabihf/release/kasarisw.bin"

arm-none-eabi-objcopy -O binary "$ELF_PATH" "$BIN_PATH"
ls -lh "$BIN_PATH"

echo "Flashing..."
dfu-util -a 0 -s 0x08000000:leave -D "$BIN_PATH"

echo "Done. View logs: probe-rs attach --chip STM32F722RETx"

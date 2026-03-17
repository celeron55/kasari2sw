# Kasari2sw: Spinning Robot with LIDAR Navigation

## Overview
Kasari2sw is a fork of the kasarisw project which used a repurposed
fixed-mounted LDS02RR LIDAR with fake encoder signal to enable autonomous
navigation and object following/wall avoidance in a robot which spins its body
(500-2000 RPM), measures rotation via ADXL373 accelerometer, and achieves
directional movement through differential motor RPM modulation. It used an
ESP-WROOM-32 module to run the firmware.

Kasari2sw upgrades the LIDAR to a Benewake TFA300 (UART interface) and supports
two MCU platforms: STM32F722RET6 (Diatone Mamba F722APP WiFi FC) and RP2350
(Raspberry Pi Pico 2 W). The STM32 firmware completely replaces Betaflight.

- **Firmware**: Multi-platform (STM32F722 or RP2350), handles sensors, motion planning, WiFi event streaming.
- **Simulator**: PC GUI (egui) for log replay or virtual world simulation; visualizes LIDAR points, detections, behavior and collisions.
- **Shared Logic**: LIDAR/accel processing, binning (90 bins, 4° each), object detection (walls/objects via averages/dips).
- **Python GUI**: `event_monitor.py` for real-time monitoring, logging (JSON), controls over TCP.
- **Goals**: Compare real vs. simulated behavior; optimize LIDAR data collection and autonomous behavior.

The robot maintains two rotations: physical (LIDAR orientation, integrated from
RPM) and virtual (stabilized relative to the world, limited by sensor
accuracy). In the simulator, the robot is rendered with its virtual rotation
fixed (pointing +X), keeping the world mostly static but adjusted for
measurement offsets/errors—useful for analyzing behavior. It produces event
logs over WiFi (captured via `event_monitor.py`), which can be replayed in the
simulator to compare real vs. simulated behavior, visualizing LIDAR points,
detections, and planner outputs.

## Simulator Architecture Notes
The system uses a MainLogic class shared between the embedded target and the
simulator for event processing and motion planning. In simulation mode (--sim),
SimEventSource owns the authoritative MainLogic to generate events (e.g.,
Lidar, Accelerometer, Planner) and drive physics. In replay mode (log file),
FileEventSource handles event sourcing (including synthetic injections like
--inject-autonomous) and uses a ReplayMirror (wrapping MainLogic) to process
events for consistent visualization. MyApp queries the EventSource trait for
logic data (e.g., detector, plans) uniformly, avoiding duplication. Flags like
--reverse-rotation are applied to each source's logic for RPM consistency.
--listen allows controlling the simulator using the same tools used to control
the robot over wifi.

## Project Structure
- `src/platform/stm32/mod.rs`: STM32F722 firmware entrypoint.
- `src/platform/stm32/sensors.rs`: STM32 sensor tasks (TFA300 LIDAR UART parsing, ADXL373 accel SPI, RC PWM capture).
- `src/platform/rp2350/mod.rs`: RP2350 (Pico 2 W) firmware entrypoint.
- `src/shared/mod.rs`: Core logic (MainLogic, MotorModulator, event handling) - **platform-independent**.
- `src/shared/algorithm.rs`: ObjectDetector (binning, RPM from accel, detection via window averages/large changes) - **platform-independent**.
- `src/simulator/mod.rs`: Simulator entrypoint.
- `src/simulator/app.rs`: Egui app for visualization.
- `src/simulator/events.rs`: Event parsing from JSON logs.
- `src/simulator/physics.rs`: Virtual world raycasting, robot physics.
- `src/simulator/sources.rs`: Event sources (file replay or simulation).
- `tools/event_monitor.py`: Tkinter GUI for monitoring/controls/logging (outdated - unmodified copy from kasarisw ESP32 version).
- `tools/tfa300.py`: TFA300 LIDAR configuration and real-time 10kHz data plotting tool with min/max/avg visualization.
- `rust-toolchain.toml`: Specifies stable Rust toolchain (targets: thumbv7em-none-eabihf for STM32F722, thumbv8m.main-none-eabihf for RP2350).
- `.cargo/config.toml`: Cargo config with probe-rs runner, build aliases (build-pc, build-stm, run-stm, build-rp, run-rp).
- `memory.x`: Linker script for RP2350 memory layout (4MB flash, 520KB RAM, image definition sections).

## Key Algorithms
- **RPM from Accel**: `a_calibrated = accel_g - offset; omega = sqrt(|a| / r); rpm = (omega * 60) / (2π)` (r=0.0145m).
- **LIDAR Binning**: 90 bins; theta integrated from RPM/dt; distances binned with interpolation/fill.
- **Detection**: Window averages (5-15 bins); find min/max for wall/open; objects via large diffs/dips; fallback protrusion check.
- **Modulation**: Base RPM + amplitude * cos(theta - phase) for directional movement.
- **Autonomous**: Blend vectors: away from wall + toward open/object; clamp mag 0.5-1.0.

## Prerequisites
- Rust stable (configured via rust-toolchain.toml for STM32 Cortex-M7 and RP2350 Cortex-M33 targets).
- PC: No special setup for simulator.
- Embedded: Python 3 (tkinter for tools).
  - **STM32F722**: Install probe-rs for debugging/flashing:
    ```sh
    cargo install probe-rs-tools --locked
    rustup target add thumbv7em-none-eabihf
    ```
  - **RP2350**: Install elf2uf2-rs for UF2 generation:
    ```sh
    cargo install --git https://github.com/JoNil/elf2uf2-rs --force
    rustup target add thumbv8m.main-none-eabihf
    ```
- Hardware (two platform options):
  - **MCU Option 1**: STM32F722RET6 (Diatone Mamba F722APP Flight Controller with UART-to-WiFi adapter)
  - **MCU Option 2**: RP2350 (Raspberry Pi Pico 2 W with integrated CYW43439 WiFi)
  - **LIDAR**: Benewake TFA300 (UART @ 921600 baud for RP2350, 115200 for STM32)
  - **Accelerometer**: ADXL373 (SPI)
  - **Motors**: 2x ESCs (400 Hz PWM, DShot on STM32)
  - **Battery**: LiPo (ADC voltage monitoring with divider)

## Build/Run
### PC Simulator
```sh
cargo +stable build-pc  # Alias for cargo build --features pc
./target/debug/simulator [OPTIONS] [SOURCE]  # SOURCE: log file or --sim for virtual and --listen for listening to control tools
```
Options: --debug, --inject-autonomous, --lidar-distance-offset <MM>, --sim.

Profiling using flamegraph:
```sh
cargo +stable flamegraph -b simulator --features=pc -- --sim --headless --end-time 100
flamegraph --perfdata perf.data --image-width 4096
```

### Embedded Firmware (STM32F722)
```sh
cargo build-stm  # Alias for cargo build --target thumbv7em-none-eabihf --features stm32
cargo run-stm    # Alias for cargo run --target thumbv7em-none-eabihf --features stm32 --bin kasarisw
                 # Uses probe-rs to flash and run on STM32F722
```

**Note**: Logs are output via defmt over RTT (Real-Time Transfer). View with:
```sh
probe-rs attach --chip STM32F722RETx
```

**Current Status**: Minimal firmware boots and logs heartbeat. Pinout identified from Betaflight MAMBAF722_I2C target. Hardware modification planned (desolder OSD chip, rewire SPI2 to ADXL373). Ready to implement:
- Motor DShot outputs (PC8, PC9)
- LIDAR UART RX (TFA300 on UART2 RX=PA3)
- WiFi adapter UART (UART4 TX=PA0, RX=PA1)
- Accelerometer SPI (ADXL373 on SPI2: CS=PB12, SCK=PB13, MISO=PB14, MOSI=PB15)
- RC receiver input capture (UART1 RX=PB7 with TIM4_CH2)
- Battery voltage ADC (PC1)

### Embedded Firmware (RP2350)
Copy FYW43 firmware to `cyw43-firmware` so that you have this (e.g. from
https://github.com/ninjasource/rp-pico2w-examples):
```
cyw43-firmware
|-- 43439A0.bin
|-- 43439A0_btfw.bin
`-- 43439A0_clm.bin
```

```sh
cargo build-rp  # Alias for cargo build --target thumbv8m.main-none-eabihf --features rp2350
                # Builds firmware for Raspberry Pi Pico 2 W (RP2350)
```

**Flashing via UF2 Bootloader**:
1. Hold BOOTSEL button while plugging in the Pico 2 W (enters bootloader mode)
2. Device will mount as a USB mass storage device
3. Generate and copy UF2 file:
```sh
# Install latest elf2uf2-rs with RP2350 support
cargo install --git https://github.com/JoNil/elf2uf2-rs --force

# Convert ELF to UF2 with correct family ID
elf2uf2-rs convert --family rp2350-arm-s \
  target/thumbv8m.main-none-eabihf/debug/kasarisw-rp2350 \
  target/thumbv8m.main-none-eabihf/debug/kasarisw-rp2350.uf2

# Copy to mounted Pico (path may vary)
cp target/thumbv8m.main-none-eabihf/debug/kasarisw-rp2350.uf2 /run/media/$USER/RP2350/
sync
```
4. Pico will automatically reboot and run the firmware

**Alternative: Direct deployment** (requires udev rules):
```sh
elf2uf2-rs deploy --family rp2350-arm-s \
  target/thumbv8m.main-none-eabihf/debug/kasarisw-rp2350
```

**Hardware**: Raspberry Pi Pico 2 W
- **MCU**: RP2350 (dual Cortex-M33 @ 150MHz, 520KB SRAM, 4MB flash)
- **WiFi**: Integrated CYW43439 (2.4GHz 802.11n)
- **LIDAR**: TFA300 on UART1 (GP5 RX @ 921600 baud)
- **Accelerometer**: ADXL373 on SPI0 (GP16-19)
- **Motors**: 2x PWM @ 400Hz (GP0-1)
- **RC Receiver**: PWM input capture on GP10
- **Battery Monitor**: ADC on GP26 (10kΩ/1kΩ divider)
- **Test LED**: GP21 (for firmware verification)

**Current Status**: Phase 3 complete - minimal firmware boots successfully and blinks LED on GP21. Ready to implement peripheral tasks (Phase 5):
- LIDAR UART reading at 10kHz
- Motor PWM control (400Hz, bidirectional ESC)
- Accelerometer SPI reading at 100Hz
- RC receiver PWM input capture
- Battery voltage ADC monitoring
- WiFi integration (Phase 6) with CYW43439 for event streaming

## Monitoring/Control
- **Note**: Monitoring tools are outdated and from the ESP32 version. Need updating for STM32F722.
- Events: Binary (tag XOR 0x5555 + payload) over TCP; JSON in logs.

## Hardware Pinout (STM32F722 - Mamba F722APP)

**Board**: Diatone Mamba F722APP MK1 (succeeds F722S)
**MCU**: STM32F722RET6
**ESC**: Mamba F50_BL32 (BLHeli_32, DShot 300/600/1200, UART6 RX telemetry)
**Betaflight Target**: MAMBAF722_I2C
**Config Source**: `local/betaflight/src/config/configs/MAMBAF722_I2C/config.h`

### Hardware Modifications

**OSD Chip Removal (MAX7456/AT7456E):**
- The onboard OSD chip is desoldered to free up SPI2 pins
- SPI2 pins are rewired to connect external ADXL373 accelerometer
- This modification is required as the robot doesn't need video OSD functionality
- The freed SPI2 bus provides dedicated hardware SPI for the accelerometer

**AT7456E Chip Pin Numbers** (for desoldering reference):
- Pin 8:  CS (connect to ADXL373 CS)
- Pin 9:  SDIN/MOSI (connect to ADXL373 SDI)
- Pin 10: SCLK (connect to ADXL373 SCLK)
- Pin 11: SDOUT/MISO (connect to ADXL373 SDO)

**Note**: ADXL373 VDD (3.3V) and GND are connected to marked power pins elsewhere on the FC board, not from the OSD chip location.

### Peripherals

**Motors (4x PWM outputs - using 2 for differential drive)**:
- MOTOR1: PC8 (TIM3_CH3)
- MOTOR2: PC9 (TIM3_CH4)
- MOTOR3: PA8 (TIM1_CH1)
- MOTOR4: PA9 (TIM1_CH2)

**UARTs** (board default connections):
- UART1: TX=PB6, RX=PB7 (Receiver port - **RC receiver input**, PB7 also has TIM4_CH2)
- UART2: TX=PA2, RX=PA3 (Vacant - **LIDAR TFA300 on RX**)
- UART3: TX=PB10, RX=PB11 (VTX port - **available**, no video transmitter)
- UART4: TX=PA0, RX=PA1 (**WiFi adapter - connected on board**)
- UART5: TX=PC12, RX=PD2 (F.Port - **available** unless using F.Port)
- UART6: TX=PC6, RX=PC7 (RX connected to ESC for telemetry, TX destination unknown)

**SPI**:
- SPI1 (Onboard MPU6000 gyro): CS=PA4, SCK=PA5, MISO=PA6, MOSI=PA7, EXTI=PC4
- SPI2 (External ADXL373 accelerometer): CS=PB12, SCK=PB13, MISO=PB14, MOSI=PB15
  - Originally connected to MAX7456 OSD (desoldered)
  - Rewired to ADXL373 accelerometer module
- SPI3 (Onboard W25Q128FV flash): CS=PA15, SCK=PC10, MISO=PC11, MOSI=PB5

**I2C**:
- I2C1: SCL=PB8, SDA=PB9

**ADC (ADC3)**:
- VBAT: PC1
- RSSI: PC2
- CURR: PC3

**GPIO**:
- LED0: PC15
- LED1: PC14
- BEEPER: PB2 (inverted)
- LED_STRIP: PB3
- PINIO1: PB0

**Onboard Hardware**:
- Gyroscope: MPU6000 (SPI1) - unused
- OSD: AT7456E/MAX7456 (SPI2) - **REMOVED** (desoldered)
- Flash: W25Q128FV 128Mbit (SPI3)
- Barometer: None

**External Hardware**:
- Accelerometer: ADXL373 (SPI2) - wired to freed OSD pins
- LIDAR: Benewake TFA300 (UART2 RX=PA3)
- RC Receiver: 1 channel on UART1 RX=PB7 (TIM4_CH2 for input capture)
- Motors: 2x ESCs via DShot (PC8, PC9)
- WiFi: UART-to-WiFi adapter (UART4)

**TFA300 LIDAR Connector Pinout** (JST GH 6-pin):
- Blue: UART_Rx (connects to STM32 TX - unused)
- Brown: UART_Tx (connects to STM32 RX - UART2 RX=PA3)
- White: CAN_L (unused)
- Green: CAN_H (unused)
- Red: VCC (5V ±10%)
- Black: GND

**RC Receiver Wiring**:
- Signal: Connect to PB7 (UART1 RX pad, will use TIM4_CH2 for input capture)
- VCC: 5V from FC
- GND: GND from FC
- Note: For simple on/off detection, threshold will be at ~1.5ms (50% of 1-2ms range)

**ADXL373 Accelerometer Wiring**:
- CS:   PB12 (wire to AT7456E pin 8 pad)
- SCLK: PB13 (wire to AT7456E pin 10 pad)
- SDO:  PB14 (wire to AT7456E pin 11 pad, MISO from STM32 perspective)
- SDI:  PB15 (wire to AT7456E pin 9 pad, MOSI from STM32 perspective)
- VDD:  3.3V (from marked power pin on FC)
- GND:  GND (from marked ground pin on FC)
- INT1: Not connected (optional interrupt pin)
- INT2: Not connected (optional interrupt pin)

## Dependencies (Cargo.toml)
- **Shared** (platform-independent): embassy-sync, ringbuffer, static_cell, arrayvec, num-traits, libm, critical-section.
- **Embedded** (ARM Cortex-M): embassy-executor, embassy-time, cortex-m, cortex-m-rt, defmt, defmt-rtt, panic-probe, embedded-hal, portable-atomic.
- **STM32F722**: embassy-stm32, embassy-net, alloc-cortex-m.
- **RP2350**: embassy-rp, cyw43, cyw43-pio, embedded-alloc.
- **Simulator** (PC): clap, eframe, egui_plot, serde_json, rand, ctrlc, crossbeam-channel.

## Event Format
- Lidar: [ts, d1..d4] (mm).
  - TODO: Let's modify the Lidar event (all platforms + shared logic +
    simulator) to hold only a single measurement to match the TFA300. No
    backwards compatibility with the old event format. Or is a single sample
    per event too little to support 10kHz sampling? Should we make it a
    10-sample event to reduce event rate?
- Accel: [ts, ay, az] (G).
- Receiver: [ts, ch, pulse|null] (us).
- Vbat: [ts, voltage] (V).
- WifiControl: [ts, mode, r, m, t].
- Planner: [ts, rotation_speed, movement_x, movement_y, cw_x, cw_y, os_x, os_y, op_x, op_y, theta, rpm].

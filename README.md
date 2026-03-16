# Kasari2sw: Spinning Robot with LIDAR Navigation

## Overview
Kasari2sw is a fork of the kasarisw project which used a repurposed
fixed-mounted LDS02RR LIDAR with fake encoder signal to enable autonomous
navigation and object following/wall avoidance in a robot which spins its body
(500-2000 RPM), measures rotation via ADXL373 accelerometer, and achieves
directional movement through differential motor RPM modulation. It used an
ESP-WROOM-32 module to run the firmware.

Kasari2sw upgrades the LIDAR to a Benewake TFA300 (UART interface) and the MCU
to a STM32F722RET6, more specifically the Diatone Mamba F722APP WiFi Flight
Controller (FC), which was originally designed to run Betaflight. This firmware
completely replaces Betaflight and changes the function and purpose of the FC
board.

- **Firmware**: STM32F7-based, handles sensors, motion planning, UART event streaming (the FC integrates a UART-to-WiFi adapter).
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
- `src/embedded.rs`: STM32F7 firmware entrypoint.
- `src/sensors.rs`: Sensor tasks (TFA300 LIDAR UART parsing, ADXL373 accel SPI, RC PWM capture).
- `src/shared/mod.rs`: Core logic (MainLogic, MotorModulator, event handling) - **platform-independent**.
- `src/shared/algorithm.rs`: ObjectDetector (binning, RPM from accel, detection via window averages/large changes) - **platform-independent**.
- `src/simulator/mod.rs`: Simulator entrypoint.
- `src/simulator/app.rs`: Egui app for visualization.
- `src/simulator/events.rs`: Event parsing from JSON logs.
- `src/simulator/physics.rs`: Virtual world raycasting, robot physics.
- `src/simulator/sources.rs`: Event sources (file replay or simulation).
- `tools/event_monitor.py`: Tkinter GUI for monitoring/controls/logging.
- `tools/download_events.py`: CLI for downloading events from robot's flash storage.
- `tools/parse_events.py`: CLI binary event parser to JSON.
- `tools/combat_wizard.py`: Tkinter GUI for combat workflow. Combines event_monitor.py and download_events.py functionalities into one tool.
- `rust-toolchain.toml`: Specifies stable Rust toolchain (target: thumbv7em-none-eabihf for STM32F722).
- `.cargo/config.toml`: Cargo config with probe-rs runner, build aliases (build-pc, build-stm, run-stm).

## Key Algorithms
- **RPM from Accel**: `a_calibrated = accel_g - offset; omega = sqrt(|a| / r); rpm = (omega * 60) / (2π)` (r=0.0145m).
- **LIDAR Binning**: 90 bins; theta integrated from RPM/dt; distances binned with interpolation/fill.
- **Detection**: Window averages (5-15 bins); find min/max for wall/open; objects via large diffs/dips; fallback protrusion check.
- **Modulation**: Base RPM + amplitude * cos(theta - phase) for directional movement.
- **Autonomous**: Blend vectors: away from wall + toward open/object; clamp mag 0.5-1.0.

## Prerequisites
- Rust stable (configured via rust-toolchain.toml for STM32 ARM Cortex-M7 target).
- PC: No special setup for simulator.
- Embedded: Python 3 (tkinter for tools); install probe-rs for flashing:
  ```sh
  cargo install probe-rs-tools --locked
  rustup target add thumbv7em-none-eabihf
  ```
- Hardware:
  - **MCU**: STM32F722RET6 (Diatone Mamba F722APP Flight Controller)
  - **LIDAR**: Benewake TFA300 (UART @ 115200 baud, no encoder needed)
  - **Accelerometer**: ADXL373 (SPI)
  - **Motors**: 2x ESCs (400 Hz PWM)
  - **Battery**: LiPo (ADC voltage monitoring)
  - **WiFi**: FC-integrated UART-to-WiFi adapter

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

## Monitoring/Control
- GUI: `python tools/event_monitor.py` (connect to 192.168.2.1:8080 AP or STA IP:8080).
  - Features: View sensors/planner, toggle modes (1=manual sliders, 2=autonomous), auto-log JSON.
- CLI Download: `python tools/download_events.py [IP:PORT]` (defaults to
  192.168.1.248:8081; downloads flash-stored events, splits into JSON logs).
- CLI: `nc IP 8080 | python tools/parse_events.py > log.json`.
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
- **Embedded** (STM32): embassy-stm32, embassy-executor, embassy-time, cortex-m, cortex-m-rt, alloc-cortex-m, defmt, defmt-rtt, panic-probe, embedded-hal.
- **Simulator** (PC): clap, eframe, egui_plot, serde_json, rand, ctrlc, crossbeam-channel.

## Event Format
- Lidar: [ts, d1..d4] (mm).
- Accel: [ts, ay, az] (G).
- Receiver: [ts, ch, pulse|null] (us).
- Vbat: [ts, voltage] (V).
- WifiControl: [ts, mode, r, m, t].
- Planner: [ts, rotation_speed, movement_x, movement_y, cw_x, cw_y, os_x, os_y, op_x, op_y, theta, rpm].

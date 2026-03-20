use embassy_stm32::usart::RingBufferedUartRx;
use embassy_stm32::gpio::{Output, Input};
use embassy_sync::pubsub::Publisher;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Timer;
use log::info;

use kasarisw::shared::kasari::InputEvent;

// TFA300 LIDAR Protocol Constants
pub const PACKET_SIZE: usize = 9; // 9 bytes for TFA300 (was 22 for LDS02RR)
pub const HEAD_BYTE_1: u8 = 0x59; // First header byte
pub const HEAD_BYTE_2: u8 = 0x59; // Second header byte
pub const LIDAR_DISTANCE_OFFSET: f32 = 36.0; // Distance offset based on mounting position (may need retuning)

// Note: No LIDAR_ENCODER_HZ needed! TFA300 doesn't require encoder signal (major simplification)

pub struct ParsedPacket {
    pub distance: u16, // mm (TFA300 gives 1 value per packet)
}

/// Parse TFA300 9-byte UART packet
/// Format: [0x59, 0x59, DIST_L, DIST_H, STRENGTH_L, STRENGTH_H, 0x00, 0x00, CHECKSUM]
pub fn parse_packet(packet: &[u8]) -> Option<ParsedPacket> {
    if packet.len() != PACKET_SIZE || packet[0] != HEAD_BYTE_1 || packet[1] != HEAD_BYTE_2 {
        return None;
    }

    // Distance: little-endian u16 (mm)
    let distance = (packet[3] as u16) << 8 | (packet[2] as u16);

    // Signal strength: little-endian u16
    let _strength = (packet[5] as u16) << 8 | (packet[4] as u16);

    // Checksum: sum of bytes 0-7, masked to lower 8 bits
    let checksum = (packet[0..8].iter().map(|&b| b as u16).sum::<u16>() & 0xFF) as u8;
    if checksum != packet[8] {
        return None;
    }

    Some(ParsedPacket { distance })
}

/// Compute TFA300 checksum (simple sum of bytes, lower 8 bits)
#[allow(dead_code)]
pub fn compute_checksum(data: &[u8]) -> u8 {
    (data.iter().map(|&b| b as u16).sum::<u16>() & 0xFF) as u8
}

/// LIDAR publisher task - reads TFA300 packets from UART2 and publishes Lidar events
#[embassy_executor::task]
pub async fn lidar_publisher(
    mut uart_rx: RingBufferedUartRx<'static>,
    publisher: Publisher<'static, CriticalSectionRawMutex, InputEvent, 64, 3, 6>,
) {
    info!("LIDAR task started");

    let mut buffer = [0u8; PACKET_SIZE * 16]; // Buffer for multiple packets
    let mut pos = 0usize;
    let mut samples = [0.0f32; 10];
    let mut sample_idx = 0usize;
    let mut packet_count = 0u32;
    let mut error_count = 0u32;

    loop {
        // Read available data into buffer
        match uart_rx.read(&mut buffer[pos..]).await {
            Ok(n) if n > 0 => {
                pos += n;

                // Process complete packets
                while pos >= PACKET_SIZE {
                    // Look for header bytes
                    if buffer[0] == HEAD_BYTE_1 && buffer[1] == HEAD_BYTE_2 {
                        if let Some(parsed) = parse_packet(&buffer[0..PACKET_SIZE]) {
                            // Apply distance offset
                            let d = parsed.distance as f32 + LIDAR_DISTANCE_OFFSET;
                            samples[sample_idx] = d.max(0.0);
                            sample_idx += 1;
                            packet_count += 1;

                            // Publish batch of 10 samples
                            if sample_idx >= 10 {
                                let ts = embassy_time::Instant::now().as_micros() as u64;
                                let event = InputEvent::Lidar(ts, samples);
                                publisher.publish_immediate(event);
                                sample_idx = 0;
                            }
                        } else {
                            error_count += 1;
                        }
                        // Shift buffer past this packet
                        buffer.copy_within(PACKET_SIZE..pos, 0);
                        pos -= PACKET_SIZE;
                    } else {
                        // Bad header, skip one byte to resync
                        buffer.copy_within(1..pos, 0);
                        pos -= 1;
                        error_count += 1;
                    }
                }

                // Log stats periodically
                if packet_count > 0 && packet_count % 10000 == 0 {
                    info!("LIDAR: {} packets, {} errors", packet_count, error_count);
                }
            }
            Ok(_) => {
                // Zero bytes read, continue
            }
            Err(_e) => {
                // UART error, reset buffer
                pos = 0;
                error_count += 1;
            }
        }
    }
}

// ============================================================================
// ADXL373 ACCELEROMETER - Bit-bang SPI Driver
// ============================================================================
//
// Pin assignments (uses available pads on Mamba F722APP):
// - CS:   PD2  (RX5 pad)
// - SCK:  PB3  (LED pad)
// - MISO: PC2  (RSSI pad, ADXL373 SDO)
// - MOSI: PC12 (TX5/F.Port pad, ADXL373 SDI)
//
// ADXL373 uses SPI mode 0 (CPOL=0, CPHA=0):
// - Clock idle low
// - Data sampled on rising edge
// - Data shifted on falling edge

// ADXL373 register addresses
const ADXL373_REG_DEVID_AD: u8 = 0x00;   // Device ID (should read 0xAD)
const ADXL373_REG_YDATA_H: u8 = 0x0A;    // Y-axis data high byte
const ADXL373_REG_POWER_CTL: u8 = 0x3F;  // Power control register

// ADXL373 constants
const ADXL373_DEVID_AD: u8 = 0xAD;       // Expected device ID
const ADXL373_G_PER_LSB: f32 = 0.1;      // 100 mg/LSB for ±200g range

/// ADXL373 bit-bang SPI driver
pub struct Adxl373BitBang<'a> {
    cs: Output<'a>,
    sck: Output<'a>,
    mosi: Output<'a>,
    miso: Input<'a>,
}

impl<'a> Adxl373BitBang<'a> {
    pub fn new(
        cs: Output<'a>,
        sck: Output<'a>,
        mosi: Output<'a>,
        miso: Input<'a>,
    ) -> Self {
        Self { cs, sck, mosi, miso }
    }

    /// Small delay for bit-bang timing (~10 kHz SPI clock)
    #[inline(always)]
    fn delay(&self) {
        // ~50 cycles at 216 MHz ≈ 230 ns, gives ~2 MHz SPI clock (plenty fast for 100 Hz sampling)
        cortex_m::asm::delay(50);
    }

    /// Transfer a single byte over SPI (full duplex)
    fn transfer_byte(&mut self, tx: u8) -> u8 {
        let mut rx: u8 = 0;

        for i in (0..8).rev() {
            // Set MOSI
            if (tx >> i) & 1 == 1 {
                self.mosi.set_high();
            } else {
                self.mosi.set_low();
            }

            self.delay();

            // Rising edge - sample MISO
            self.sck.set_high();
            self.delay();

            if self.miso.is_high() {
                rx |= 1 << i;
            }

            // Falling edge
            self.sck.set_low();
        }

        rx
    }

    /// Read a single register
    pub fn read_reg(&mut self, addr: u8) -> u8 {
        self.cs.set_low();
        self.delay();

        // Address byte: bit 0 = R/W (1=read), bits 7:1 = address
        self.transfer_byte((addr << 1) | 1);
        let value = self.transfer_byte(0x00);

        self.delay();
        self.cs.set_high();

        value
    }

    /// Write a single register
    pub fn write_reg(&mut self, addr: u8, value: u8) {
        self.cs.set_low();
        self.delay();

        // Address byte: bit 0 = R/W (0=write), bits 7:1 = address
        self.transfer_byte((addr << 1) | 0);
        self.transfer_byte(value);

        self.delay();
        self.cs.set_high();
    }

    /// Read multiple consecutive registers (burst read)
    pub fn read_burst(&mut self, addr: u8, buf: &mut [u8]) {
        self.cs.set_low();
        self.delay();

        // Address byte with read bit
        self.transfer_byte((addr << 1) | 1);

        // Read data bytes
        for byte in buf.iter_mut() {
            *byte = self.transfer_byte(0x00);
        }

        self.delay();
        self.cs.set_high();
    }

    /// Initialize ADXL373 and return true if device responds correctly
    pub fn init(&mut self) -> bool {
        // Ensure CS is high and clock is low initially
        self.cs.set_high();
        self.sck.set_low();
        self.mosi.set_low();

        // Small delay for power-on
        cortex_m::asm::delay(10_000);

        // Read device ID
        let devid = self.read_reg(ADXL373_REG_DEVID_AD);
        info!("ADXL373 DEVID_AD = {:#04x} (expected {:#04x})", devid, ADXL373_DEVID_AD);

        if devid != ADXL373_DEVID_AD {
            return false;
        }

        // Configure power control: measurement mode
        // POWER_CTL (0x3F): bits 1:0 = mode (0b10 = measurement mode)
        // Full bandwidth mode, no high-pass filter
        self.write_reg(ADXL373_REG_POWER_CTL, 0b00000010);

        // Small delay for mode change
        cortex_m::asm::delay(10_000);

        true
    }

    /// Read Y-axis acceleration in G (only axis needed for RPM calculation)
    pub fn read_y(&mut self) -> f32 {
        // Read 2 bytes: YDATA_H, YDATA_L
        let mut buf = [0u8; 2];
        self.read_burst(ADXL373_REG_YDATA_H, &mut buf);

        // Parse 12-bit signed value (upper 12 bits of 16-bit register pair)
        // Data is MSB first, 12-bit left-justified
        let y_raw = ((buf[0] as i16) << 8 | buf[1] as i16) >> 4;

        // Convert to G (100 mg/LSB for ±200g range)
        y_raw as f32 * ADXL373_G_PER_LSB
    }
}

/// Accelerometer publisher task - reads ADXL373 at 100 Hz and publishes events
#[embassy_executor::task]
pub async fn accel_publisher(
    cs: Output<'static>,
    sck: Output<'static>,
    mosi: Output<'static>,
    miso: Input<'static>,
    publisher: Publisher<'static, CriticalSectionRawMutex, InputEvent, 64, 3, 6>,
) {
    info!("Accelerometer task started");

    let mut adxl = Adxl373BitBang::new(cs, sck, mosi, miso);

    // Initialize sensor
    if !adxl.init() {
        info!("ADXL373 initialization failed - task exiting");
        return;
    }
    info!("ADXL373 initialized successfully");

    let mut sample_count = 0u32;

    loop {
        let y_g = adxl.read_y();

        // Publish event (only Y axis used for centripetal acceleration / RPM calculation)
        // Z axis unused, filled with 0
        let ts = embassy_time::Instant::now().as_micros() as u64;
        let event = InputEvent::Accelerometer(ts, -y_g, 0.0);
        publisher.publish_immediate(event);

        // Log stats periodically
        sample_count += 1;
        if sample_count % 1000 == 0 {
            info!("ADXL373: {} samples, Y={:.1} G", sample_count, y_g);
        }

        // 100 Hz sampling
        Timer::after_millis(10).await;
    }
}

// ============================================================================
// TODO: RC Receiver Task (TIM4_CH2 input capture on PB7)
// ============================================================================

// ============================================================================
// TODO: Battery Voltage Monitor Task (ADC on PC1)
// ============================================================================

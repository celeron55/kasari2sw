#![allow(dead_code)]
#![allow(non_upper_case_globals)]
#![allow(unused_imports)]

use embassy_stm32::peripherals;
use log::info;

pub const DSHOT300_HZ: u32 = 300_000;
pub const DSHOT600_HZ: u32 = 600_000;
pub const DSHOT150_HZ: u32 = 150_000;

pub const MOTOR_BIT_0: u32 = 7;
pub const MOTOR_BIT_1: u32 = 14;
pub const MOTOR_BITLENGTH: u32 = 20;
pub const DSHOT_DMA_BUFFER_SIZE: usize = 18;

pub const DSHOT_MOTOR_STOP: u16 = 0;
pub const DSHOT_MOTOR_MIN: u16 = 48;
pub const DSHOT_MOTOR_MAX: u16 = 2047;

const TIM3_BASE: u32 = 0x4000_0400;
const TIM3_CR1: *mut u32 = (TIM3_BASE + 0x00) as *mut u32;
const TIM3_CR2: *mut u32 = (TIM3_BASE + 0x04) as *mut u32;
const TIM3_SMCR: *mut u32 = (TIM3_BASE + 0x08) as *mut u32;
const TIM3_DIER: *mut u32 = (TIM3_BASE + 0x0C) as *mut u32;
const TIM3_SR: *mut u32 = (TIM3_BASE + 0x10) as *mut u32;
const TIM3_EGR: *mut u32 = (TIM3_BASE + 0x14) as *mut u32;
const TIM3_CCMR1: *mut u32 = (TIM3_BASE + 0x18) as *mut u32;
const TIM3_CCMR2: *mut u32 = (TIM3_BASE + 0x1C) as *mut u32;
const TIM3_CCER: *mut u32 = (TIM3_BASE + 0x20) as *mut u32;
const TIM3_CNT: *mut u32 = (TIM3_BASE + 0x24) as *mut u32;
const TIM3_PSC: *mut u32 = (TIM3_BASE + 0x28) as *mut u32;
const TIM3_ARR: *mut u32 = (TIM3_BASE + 0x2C) as *mut u32;
const TIM3_CCR3: *mut u32 = (TIM3_BASE + 0x3C) as *mut u32;
const TIM3_CCR4: *mut u32 = (TIM3_BASE + 0x40) as *mut u32;

const RCC_BASE: u32 = 0x4002_3800;
const RCC_APB1ENR: *mut u32 = (RCC_BASE + 0x40) as *mut u32;
const RCC_APB1ENR_TIM3EN: u32 = 1 << 1;

const GPIOC_BASE: u32 = 0x4002_0800;
const GPIOC_MODER: *mut u32 = (GPIOC_BASE + 0x00) as *mut u32;
const GPIOC_AFRL: *mut u32 = (GPIOC_BASE + 0x20) as *mut u32;
const GPIOC_AFRH: *mut u32 = (GPIOC_BASE + 0x24) as *mut u32;
const GPIOC_OSPEEDR: *mut u32 = (GPIOC_BASE + 0x08) as *mut u32;

const RCC_AHB1ENR: *mut u32 = (RCC_BASE + 0x30) as *mut u32;
const RCC_AHB1ENR_GPIOCEN: u32 = 1 << 2;

const TIM3_AF: u32 = 2;

const CR1_CEN: u32 = 1 << 0;
const CR1_UDIS: u32 = 1 << 1;
const CR1_URS: u32 = 1 << 2;

const CCMR_OCxM_PWM1: u32 = 0b110 << 4;
const CCMR_OCxPE: u32 = 1 << 3;
const CCMR_OCxFE: u32 = 1 << 2;
const CCMR_CC1S: u32 = 0b00 << 0;
const CCMR_CC2S: u32 = 0b00 << 8;

const CCER_CC3E: u32 = 1 << 8;
const CCER_CC3P: u32 = 1 << 9;
const CCER_CC4E: u32 = 1 << 12;
const CCER_CC4P: u32 = 1 << 13;

const SR_UIF: u32 = 1 << 0;
const EGR_UG: u32 = 1 << 0;

pub struct DShot;

impl DShot {
    pub fn new() -> Self {
        Self
    }

    pub fn init(&mut self, protocol_hz: u32) {
        let timer_clock = 108_000_000;
        let prescaler = ((timer_clock / (MOTOR_BITLENGTH * protocol_hz)) - 1) as u16;
        let period = (MOTOR_BITLENGTH - 1) as u16;

        unsafe {
            let rcc_ahb1enr = core::ptr::read_volatile(RCC_AHB1ENR);
            core::ptr::write_volatile(RCC_AHB1ENR, rcc_ahb1enr | RCC_AHB1ENR_GPIOCEN);

            let rcc_apb1enr = core::ptr::read_volatile(RCC_APB1ENR);
            core::ptr::write_volatile(RCC_APB1ENR, rcc_apb1enr | RCC_APB1ENR_TIM3EN);

            core::ptr::write_volatile(TIM3_CR1, 0);

            core::ptr::write_volatile(TIM3_PSC, prescaler as u32);
            core::ptr::write_volatile(TIM3_ARR, period as u32);

            core::ptr::write_volatile(TIM3_CCMR2, 0x6868);

            let mut moder = core::ptr::read_volatile(GPIOC_MODER);
            moder = (moder & 0xFFF0_0000) | 0x000A_0000;
            core::ptr::write_volatile(GPIOC_MODER, moder);

            let mut afrh = core::ptr::read_volatile(GPIOC_AFRH);
            afrh = (afrh & 0xFFFF_FFF0) | 0x0000_0022;
            core::ptr::write_volatile(GPIOC_AFRH, afrh);

            let mut ospeedr = core::ptr::read_volatile(GPIOC_OSPEEDR);
            ospeedr = (ospeedr & 0xFFF0_0000) | 0x000A_0000;
            core::ptr::write_volatile(GPIOC_OSPEEDR, ospeedr);

            core::ptr::write_volatile(TIM3_CCER, CCER_CC3E | CCER_CC4E);

            core::ptr::write_volatile(TIM3_CCR3, 0);
            core::ptr::write_volatile(TIM3_CCR4, 0);

            core::ptr::write_volatile(TIM3_EGR, EGR_UG);

            core::ptr::write_volatile(TIM3_CR1, CR1_URS | CR1_CEN);

            info!(
                "  GPIOC_MODER = 0x{:08X}",
                core::ptr::read_volatile(GPIOC_MODER)
            );
            info!(
                "  GPIOC_AFRH = 0x{:08X}",
                core::ptr::read_volatile(GPIOC_AFRH)
            );
            info!(
                "  GPIOC_OSPEEDR = 0x{:08X}",
                core::ptr::read_volatile(GPIOC_OSPEEDR)
            );
            info!(
                "  TIM3_CCMR2 = 0x{:08X}",
                core::ptr::read_volatile(TIM3_CCMR2)
            );
            info!(
                "  TIM3_CCER = 0x{:08X} (CC3E=bit8, CC4E=bit12)",
                core::ptr::read_volatile(TIM3_CCER)
            );
            info!("  TIM3_CR1 = 0x{:08X}", core::ptr::read_volatile(TIM3_CR1));
        }

        info!(
            "DShot initialized: prescaler={}, period={}, protocol_hz={}",
            prescaler, period, protocol_hz
        );
    }

    pub fn send_frame(&mut self, throttle1: u16, throttle2: u16) {
        let frame1 = throttle_to_dshot_frame(throttle1, false);
        let frame2 = throttle_to_dshot_frame(throttle2, false);

        self.send_raw_frame(frame1, frame2);
    }

    fn send_raw_frame(&mut self, frame1: u16, frame2: u16) {
        unsafe {
            for i in 0..16 {
                let bit1 = (frame1 >> (15 - i)) & 1;
                let bit2 = (frame2 >> (15 - i)) & 1;

                let ccr3_val = if bit1 == 1 { MOTOR_BIT_1 } else { MOTOR_BIT_0 };
                let ccr4_val = if bit2 == 1 { MOTOR_BIT_1 } else { MOTOR_BIT_0 };

                core::ptr::write_volatile(TIM3_CCR3, ccr3_val);
                core::ptr::write_volatile(TIM3_CCR4, ccr4_val);
                core::ptr::write_volatile(TIM3_EGR, EGR_UG);

                while core::ptr::read_volatile(TIM3_SR) & SR_UIF == 0 {}
                core::ptr::write_volatile(TIM3_SR, 0);
            }
        }
    }

    pub fn stop(&mut self) {
        unsafe {
            let cr1 = core::ptr::read_volatile(TIM3_CR1);
            core::ptr::write_volatile(TIM3_CR1, cr1 & !CR1_CEN);
        }
    }
}

pub fn throttle_to_dshot_frame(throttle: u16, telemetry_req: bool) -> u16 {
    // DShot packet format (MSB first):
    // Bits 15-5: throttle value (11 bits)
    // Bit 4: telemetry request
    // Bits 3-0: checksum (4 bits)

    // Step 1: Create 12-bit packet with throttle and telemetry bit
    let packet = (throttle << 1) | if telemetry_req { 1 } else { 0 };

    // Step 2: Compute checksum by XORing nibbles
    let mut csum = 0u16;
    let mut csum_data = packet;
    for _ in 0..3 {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    csum &= 0xF;

    // Step 3: Shift packet left 4 bits and append checksum
    (packet << 4) | csum
}

pub fn speed_percent_to_dshot(speed_percent: f32) -> u16 {
    let throttle = ((speed_percent + 1.0) * 1000.0) as i32;
    throttle.clamp(DSHOT_MOTOR_MIN as i32, DSHOT_MOTOR_MAX as i32) as u16
}

pub fn rpm_to_dshot_throttle(rpm: f32, max_rpm: f32) -> u16 {
    let speed_percent = (rpm / max_rpm) * 100.0;
    speed_percent_to_dshot(speed_percent)
}

pub fn load_dshot_dma_buffer(dma_buffer: &mut [u32; DSHOT_DMA_BUFFER_SIZE], packet: u16) {
    for i in 0..16 {
        let bit_value = (packet >> (15 - i)) & 1;
        dma_buffer[i] = if bit_value == 1 {
            MOTOR_BIT_1
        } else {
            MOTOR_BIT_0
        };
    }
    dma_buffer[16] = 0;
    dma_buffer[17] = 0;
}

pub struct DShotTelemetryParser {
    buffer: [u8; 10],
    pos: usize,
}

impl DShotTelemetryParser {
    pub fn new() -> Self {
        Self {
            buffer: [0u8; 10],
            pos: 0,
        }
    }

    pub fn feed_byte(&mut self, byte: u8) -> Option<DShotTelemetry> {
        self.buffer[self.pos] = byte;
        self.pos = (self.pos + 1) % 10;

        if byte != 0 && self.pos == 0 {
            let temp = self.buffer[0] as i16;
            let voltage = ((self.buffer[2] as u16) | ((self.buffer[1] as u16) << 8)) as f32 / 100.0;
            let current = ((self.buffer[4] as u16) | ((self.buffer[3] as u16) << 8)) as f32 / 100.0;
            let erpm = ((self.buffer[6] as u16) | ((self.buffer[5] as u16) << 8)) as u32 * 100;
            let consumption = ((self.buffer[8] as u16) | ((self.buffer[7] as u16) << 8)) as u32;

            return Some(DShotTelemetry {
                temperature: temp,
                voltage,
                current,
                erpm,
                consumption,
            });
        }

        None
    }

    pub fn reset(&mut self) {
        self.pos = 0;
    }
}

impl Default for DShotTelemetryParser {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct DShotTelemetry {
    pub temperature: i16,
    pub voltage: f32,
    pub current: f32,
    pub erpm: u32,
    pub consumption: u32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dshot_frame_crc() {
        let frame = throttle_to_dshot_frame(1000, false);
        let crc = frame & 0x0F;

        // Extract 12-bit packet (bits 15-4)
        let packet = frame >> 4;

        // Compute expected checksum
        let mut expected_crc = 0u16;
        let mut csum_data = packet;
        for _ in 0..3 {
            expected_crc ^= csum_data;
            csum_data >>= 4;
        }
        expected_crc &= 0x0F;

        assert_eq!(crc, expected_crc);
    }

    #[test]
    fn test_disarmed_frame_is_zero_except_crc() {
        let frame = throttle_to_dshot_frame(0, false);
        // For throttle=0, telemetry=false: packet = (0 << 1) | 0 = 0
        // Checksum of 0 XOR 0 XOR 0 = 0
        assert_eq!(frame, 0x0000);
    }

    #[test]
    fn test_dshot_dma_buffer_loading() {
        let mut buffer = [0u32; DSHOT_DMA_BUFFER_SIZE];
        load_dshot_dma_buffer(&mut buffer, 0x0FFF);

        for i in 0..16 {
            assert_eq!(buffer[i], MOTOR_BIT_1, "Bit {} should be 1", i);
        }
        assert_eq!(buffer[16], 0);
        assert_eq!(buffer[17], 0);

        load_dshot_dma_buffer(&mut buffer, 0x0000);
        for i in 0..16 {
            assert_eq!(buffer[i], MOTOR_BIT_0, "Bit {} should be 0", i);
        }
    }

    #[test]
    fn test_betaflight_compatible_frame() {
        // Test case from Betaflight: throttle=500, telemetry=true
        // packet = (500 << 1) | 1 = 0x3E9
        // checksum nibbles: 0x3E9 ^ 0x3E ^ 0x3 = 0x4
        // final frame = (0x3E9 << 4) | 0x4 = 0x3E94
        let frame = throttle_to_dshot_frame(500, true);
        assert_eq!(frame, 0x3E94, "Frame should match Betaflight format");

        // Verify bit pattern: 0011 1110 1001 0100
        // which sends: 0 0 1 1  1 1 1 0  1 0 0 1  0 1 0 0 (MSB first)
    }
}

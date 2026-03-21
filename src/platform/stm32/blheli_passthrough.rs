#![allow(dead_code)]

// MSP constants
const MSP_HEADER: &[u8] = b"$M<";
const MSP_PASSTHROUGH_ESC_4WAY: u8 = 0xFF;

// 4-way interface constants
const ESC_LOCAL_ESCAPE: u8 = 0x2F;

// 4-way commands
const CMD_INTERFACE_TEST_ALIVE: u8 = 0x30;
const CMD_PROTOCOL_GET_VERSION: u8 = 0x31;
const CMD_INTERFACE_GET_NAME: u8 = 0x32;
const CMD_INTERFACE_GET_VERSION: u8 = 0x33;
const CMD_INTERFACE_EXIT: u8 = 0x34;
const CMD_DEVICE_RESET: u8 = 0x35;
const CMD_DEVICE_INIT_FLASH: u8 = 0x37;
const CMD_DEVICE_ERASE_ALL: u8 = 0x38;
const CMD_DEVICE_PAGE_ERASE: u8 = 0x39;
const CMD_DEVICE_READ: u8 = 0x3A;
const CMD_DEVICE_WRITE: u8 = 0x3B;
const CMD_DEVICE_C2CK_LOW: u8 = 0x3C;
const CMD_DEVICE_READ_EEPROM: u8 = 0x3D;
const CMD_DEVICE_WRITE_EEPROM: u8 = 0x3E;
const CMD_INTERFACE_SET_MODE: u8 = 0x3F;

// ACK codes
const ACK_OK: u8 = 0x00;
const ACK_I_UNKNOWN_ERROR: u8 = 0x01;
const ACK_I_INVALID_CMD: u8 = 0x02;
const ACK_I_INVALID_CRC: u8 = 0x03;
const ACK_I_VERIFY_ERROR: u8 = 0x04;
const ACK_D_GENERAL_ERROR: u8 = 0x08;
const ACK_I_INVALID_CHANNEL: u8 = 0x09;

// Interface modes
const IM_ARM_BLB: u8 = 4;  // BLHeli_32

// BLHeli bootloader commands
const BL_CMD_RUN: u8 = 0x00;
const BL_CMD_PROG_FLASH: u8 = 0x01;
const BL_CMD_ERASE_FLASH: u8 = 0x02;
const BL_CMD_READ_FLASH_SIL: u8 = 0x03;
const BL_CMD_VERIFY_FLASH: u8 = 0x03;
const BL_CMD_VERIFY_FLASH_ARM: u8 = 0x04;
const BL_CMD_READ_EEPROM: u8 = 0x04;
const BL_CMD_PROG_EEPROM: u8 = 0x05;
const BL_CMD_READ_SRAM: u8 = 0x06;
const BL_CMD_READ_FLASH_ATM: u8 = 0x07;
const BL_CMD_KEEP_ALIVE: u8 = 0xFD;
const BL_CMD_SET_BUFFER: u8 = 0xFE;
const BL_CMD_SET_ADDRESS: u8 = 0xFF;

// Timing
const BIT_TIME_US: u32 = 52;  // 19200 baud = 52.08us per bit
const START_BIT_DELAY_US: u32 = BIT_TIME_US + (BIT_TIME_US / 2);  // 1.5 bit times

// GPIO registers for motor pins
const GPIOA_BASE: u32 = 0x4002_0000;
const GPIOC_BASE: u32 = 0x4002_0800;
const GPIO_MODER: u32 = 0x00;
const GPIO_OTYPER: u32 = 0x04;
const GPIO_OSPEEDR: u32 = 0x08;
const GPIO_PUPDR: u32 = 0x0C;
const GPIO_IDR: u32 = 0x10;
const GPIO_ODR: u32 = 0x14;
const GPIO_BSRR: u32 = 0x18;

// Motor pin assignments
// MOTOR1/2 = PC8/PC9 (TIM3_CH3/4)
// MOTOR3/4 = PA8/PA9 (TIM1_CH1/2)
const MOTOR_PINS: [(u32, u8); 4] = [
    (GPIOC_BASE, 8),  // Motor 1
    (GPIOC_BASE, 9),  // Motor 2
    (GPIOA_BASE, 8),  // Motor 3
    (GPIOA_BASE, 9),  // Motor 4
];

// Number of ESCs
const ESC_COUNT: u8 = 4;

// Interface info
const INTERFACE_NAME: &[u8] = b"KASARI2";
const PROTOCOL_VERSION: u16 = 108;
const INTERFACE_VERSION: u16 = 1;

pub struct MspParser {
    state: MspState,
    len: u8,
    cmd: u8,
    payload: [u8; 64],
    payload_idx: usize,
    checksum: u8,
}

#[derive(Clone, Copy, PartialEq)]
enum MspState {
    Idle,
    Header1,  // Got '$'
    Header2,  // Got 'M'
    Direction, // Got '<' or '>'
    Length,
    Command,
    Payload,
    Checksum,
}

impl MspParser {
    pub fn new() -> Self {
        Self {
            state: MspState::Idle,
            len: 0,
            cmd: 0,
            payload: [0u8; 64],
            payload_idx: 0,
            checksum: 0,
        }
    }

    pub fn reset(&mut self) {
        self.state = MspState::Idle;
        self.len = 0;
        self.cmd = 0;
        self.payload_idx = 0;
        self.checksum = 0;
    }

    /// Feed a byte, returns Some(cmd) if a complete valid MSP frame was received
    pub fn feed(&mut self, byte: u8) -> Option<u8> {
        match self.state {
            MspState::Idle => {
                if byte == b'$' {
                    self.state = MspState::Header1;
                }
            }
            MspState::Header1 => {
                if byte == b'M' {
                    self.state = MspState::Header2;
                } else {
                    self.reset();
                }
            }
            MspState::Header2 => {
                if byte == b'<' {
                    self.state = MspState::Direction;
                } else {
                    self.reset();
                }
            }
            MspState::Direction => {
                self.len = byte;
                self.checksum = byte;
                self.payload_idx = 0;
                self.state = MspState::Length;
            }
            MspState::Length => {
                self.cmd = byte;
                self.checksum ^= byte;
                if self.len == 0 {
                    self.state = MspState::Checksum;
                } else {
                    self.state = MspState::Command;
                }
            }
            MspState::Command => {
                if self.payload_idx < self.len as usize && self.payload_idx < self.payload.len() {
                    self.payload[self.payload_idx] = byte;
                    self.payload_idx += 1;
                    self.checksum ^= byte;
                }
                if self.payload_idx >= self.len as usize {
                    self.state = MspState::Checksum;
                }
            }
            MspState::Checksum => {
                let valid = byte == self.checksum;
                let cmd = self.cmd;
                self.reset();
                if valid {
                    return Some(cmd);
                }
            }
            _ => {
                self.reset();
            }
        }
        None
    }
}

/// Calculate CRC16 XMODEM
fn crc16_xmodem(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// Set motor pin as output (push-pull, high speed)
unsafe fn pin_set_output(gpio_base: u32, pin: u8) {
    let moder = (gpio_base + GPIO_MODER) as *mut u32;
    let ospeedr = (gpio_base + GPIO_OSPEEDR) as *mut u32;
    let otyper = (gpio_base + GPIO_OTYPER) as *mut u32;

    // Set as output (MODER = 01)
    let mut val = core::ptr::read_volatile(moder);
    val &= !(0b11 << (pin * 2));
    val |= 0b01 << (pin * 2);
    core::ptr::write_volatile(moder, val);

    // High speed
    let mut val = core::ptr::read_volatile(ospeedr);
    val |= 0b11 << (pin * 2);
    core::ptr::write_volatile(ospeedr, val);

    // Push-pull
    let mut val = core::ptr::read_volatile(otyper);
    val &= !(1 << pin);
    core::ptr::write_volatile(otyper, val);
}

/// Set motor pin as input with pull-up
unsafe fn pin_set_input(gpio_base: u32, pin: u8) {
    let moder = (gpio_base + GPIO_MODER) as *mut u32;
    let pupdr = (gpio_base + GPIO_PUPDR) as *mut u32;

    // Set as input (MODER = 00)
    let mut val = core::ptr::read_volatile(moder);
    val &= !(0b11 << (pin * 2));
    core::ptr::write_volatile(moder, val);

    // Pull-up (PUPDR = 01)
    let mut val = core::ptr::read_volatile(pupdr);
    val &= !(0b11 << (pin * 2));
    val |= 0b01 << (pin * 2);
    core::ptr::write_volatile(pupdr, val);
}

/// Set pin high
unsafe fn pin_set_high(gpio_base: u32, pin: u8) {
    let bsrr = (gpio_base + GPIO_BSRR) as *mut u32;
    core::ptr::write_volatile(bsrr, 1 << pin);
}

/// Set pin low
unsafe fn pin_set_low(gpio_base: u32, pin: u8) {
    let bsrr = (gpio_base + GPIO_BSRR) as *mut u32;
    core::ptr::write_volatile(bsrr, 1 << (pin + 16));
}

/// Read pin state
unsafe fn pin_read(gpio_base: u32, pin: u8) -> bool {
    let idr = (gpio_base + GPIO_IDR) as *mut u32;
    (core::ptr::read_volatile(idr) & (1 << pin)) != 0
}

/// Delay in microseconds using cortex_m::asm::delay
/// Assumes 216 MHz clock
fn delay_us(us: u32) {
    // 216 cycles per microsecond
    cortex_m::asm::delay(us * 216);
}

/// Bit-bang UART transmit one byte at 19200 baud
unsafe fn suart_putc(gpio_base: u32, pin: u8, byte: u8) {
    pin_set_output(gpio_base, pin);

    // Start bit (low)
    pin_set_low(gpio_base, pin);
    delay_us(BIT_TIME_US);

    // Data bits (LSB first)
    let mut data = byte;
    for _ in 0..8 {
        if data & 1 != 0 {
            pin_set_high(gpio_base, pin);
        } else {
            pin_set_low(gpio_base, pin);
        }
        data >>= 1;
        delay_us(BIT_TIME_US);
    }

    // Stop bit (high)
    pin_set_high(gpio_base, pin);
    delay_us(BIT_TIME_US);
}

/// Bit-bang UART receive one byte at 19200 baud with timeout
/// Returns None on timeout
unsafe fn suart_getc(gpio_base: u32, pin: u8, timeout_us: u32) -> Option<u8> {
    pin_set_input(gpio_base, pin);

    // Wait for start bit (line goes low)
    let mut waited: u32 = 0;
    while pin_read(gpio_base, pin) {
        delay_us(1);
        waited += 1;
        if waited > timeout_us {
            return None;
        }
    }

    // Wait 1.5 bit times to sample in middle of first data bit
    delay_us(START_BIT_DELAY_US);

    // Read 8 data bits (LSB first)
    let mut byte: u8 = 0;
    for i in 0..8 {
        if pin_read(gpio_base, pin) {
            byte |= 1 << i;
        }
        delay_us(BIT_TIME_US);
    }

    // Skip stop bit (already high)
    // No need to wait, next byte will sync on start bit

    Some(byte)
}

/// Send buffer to ESC bootloader with CRC16
unsafe fn bl_send_buf(gpio_base: u32, pin: u8, cmd: u8, addr: u16, data: &[u8]) {
    // Send command
    suart_putc(gpio_base, pin, cmd);

    // Send address (big endian)
    suart_putc(gpio_base, pin, (addr >> 8) as u8);
    suart_putc(gpio_base, pin, addr as u8);

    // Build CRC data: cmd + addr_h + addr_l + data
    let mut crc_buf = [0u8; 260];
    crc_buf[0] = cmd;
    crc_buf[1] = (addr >> 8) as u8;
    crc_buf[2] = addr as u8;
    let len = data.len().min(256);
    crc_buf[3..3+len].copy_from_slice(&data[..len]);

    // Send data
    for &byte in &data[..len] {
        suart_putc(gpio_base, pin, byte);
    }

    // Calculate and send CRC16
    let crc = crc16_xmodem(&crc_buf[..3+len]);
    suart_putc(gpio_base, pin, (crc >> 8) as u8);
    suart_putc(gpio_base, pin, crc as u8);
}

/// Receive bootloader response
unsafe fn bl_recv_byte(gpio_base: u32, pin: u8) -> Option<u8> {
    suart_getc(gpio_base, pin, 50000)  // 50ms timeout
}

/// Initialize ESC bootloader communication
unsafe fn bl_init_esc(esc_idx: u8) -> bool {
    if esc_idx >= ESC_COUNT {
        return false;
    }

    let (gpio_base, pin) = MOTOR_PINS[esc_idx as usize];

    // Set pin high initially
    pin_set_output(gpio_base, pin);
    pin_set_high(gpio_base, pin);
    delay_us(100);

    // Send bootloader init sequence (pulse low)
    pin_set_low(gpio_base, pin);
    delay_us(500);
    pin_set_high(gpio_base, pin);
    delay_us(100000);  // 100ms wait for bootloader

    true
}

/// 4-way interface frame handler
pub struct FourWayInterface {
    current_esc: u8,
    esc_mode: u8,
}

impl FourWayInterface {
    pub fn new() -> Self {
        Self {
            current_esc: 0,
            esc_mode: IM_ARM_BLB,
        }
    }

    /// Parse and handle a 4-way command frame
    /// Returns response to send back
    pub fn handle_frame(&mut self, frame: &[u8]) -> heapless::Vec<u8, 280> {
        let response: heapless::Vec<u8, 280> = heapless::Vec::new();

        if frame.len() < 6 {
            return response;
        }

        // Frame format: ESC(0x2F) + CMD + ADDR_H + ADDR_L + PARAM_LEN + [PARAM] + CRC_H + CRC_L
        if frame[0] != ESC_LOCAL_ESCAPE {
            return response;
        }

        let cmd = frame[1];
        let addr = ((frame[2] as u16) << 8) | (frame[3] as u16);
        let param_len = frame[4] as usize;

        // Verify CRC
        let crc_idx = 5 + param_len;
        if frame.len() < crc_idx + 2 {
            return self.build_response(cmd, addr, &[], ACK_I_INVALID_CRC);
        }

        let expected_crc = crc16_xmodem(&frame[..crc_idx]);
        let received_crc = ((frame[crc_idx] as u16) << 8) | (frame[crc_idx + 1] as u16);

        if expected_crc != received_crc {
            return self.build_response(cmd, addr, &[], ACK_I_INVALID_CRC);
        }

        let params = if param_len > 0 && frame.len() >= 5 + param_len {
            &frame[5..5+param_len]
        } else {
            &[]
        };

        // Handle command
        let (resp_data, ack) = self.process_command(cmd, addr, params);
        self.build_response(cmd, addr, &resp_data, ack)
    }

    fn process_command(&mut self, cmd: u8, addr: u16, params: &[u8]) -> (heapless::Vec<u8, 256>, u8) {
        let mut data: heapless::Vec<u8, 256> = heapless::Vec::new();

        match cmd {
            CMD_INTERFACE_TEST_ALIVE => {
                (data, ACK_OK)
            }

            CMD_PROTOCOL_GET_VERSION => {
                let _ = data.push((PROTOCOL_VERSION >> 8) as u8);
                let _ = data.push(PROTOCOL_VERSION as u8);
                (data, ACK_OK)
            }

            CMD_INTERFACE_GET_NAME => {
                for &b in INTERFACE_NAME {
                    let _ = data.push(b);
                }
                (data, ACK_OK)
            }

            CMD_INTERFACE_GET_VERSION => {
                let _ = data.push((INTERFACE_VERSION >> 8) as u8);
                let _ = data.push(INTERFACE_VERSION as u8);
                (data, ACK_OK)
            }

            CMD_INTERFACE_EXIT => {
                // We don't actually exit, just ACK
                (data, ACK_OK)
            }

            CMD_INTERFACE_SET_MODE => {
                if params.len() >= 1 {
                    self.esc_mode = params[0];
                }
                (data, ACK_OK)
            }

            CMD_DEVICE_INIT_FLASH => {
                // addr contains ESC index
                self.current_esc = addr as u8;
                if self.current_esc >= ESC_COUNT {
                    return (data, ACK_I_INVALID_CHANNEL);
                }

                unsafe {
                    if bl_init_esc(self.current_esc) {
                        // Try to get device info
                        let (gpio_base, pin) = MOTOR_PINS[self.current_esc as usize];

                        // Send "get info" command to bootloader
                        // For BLHeli_32, we need to wake up the bootloader
                        suart_putc(gpio_base, pin, 0x00);  // Sync
                        delay_us(1000);

                        // Return device signature
                        let _ = data.push(0xE8);  // Device ID byte 1 (typical BLHeli_32)
                        let _ = data.push(0x04);  // Device ID byte 2
                        let _ = data.push(0x00);  // Flash size indicator
                        let _ = data.push(0x00);

                        (data, ACK_OK)
                    } else {
                        (data, ACK_D_GENERAL_ERROR)
                    }
                }
            }

            CMD_DEVICE_RESET => {
                if self.current_esc < ESC_COUNT {
                    unsafe {
                        let (gpio_base, pin) = MOTOR_PINS[self.current_esc as usize];
                        // Send reset command to bootloader
                        suart_putc(gpio_base, pin, BL_CMD_RUN);
                        delay_us(1000);
                    }
                }
                (data, ACK_OK)
            }

            CMD_DEVICE_READ => {
                if self.current_esc >= ESC_COUNT {
                    return (data, ACK_I_INVALID_CHANNEL);
                }

                let read_len = if params.len() >= 1 {
                    if params[0] == 0 { 256 } else { params[0] as usize }
                } else {
                    0
                };

                unsafe {
                    let (gpio_base, pin) = MOTOR_PINS[self.current_esc as usize];

                    // Send SET_ADDRESS command
                    suart_putc(gpio_base, pin, BL_CMD_SET_ADDRESS);
                    suart_putc(gpio_base, pin, 0x00);
                    suart_putc(gpio_base, pin, (addr >> 8) as u8);
                    suart_putc(gpio_base, pin, addr as u8);

                    // Wait for ACK
                    if let Some(ack) = bl_recv_byte(gpio_base, pin) {
                        if ack != BL_CMD_SET_ADDRESS {
                            return (data, ACK_D_GENERAL_ERROR);
                        }
                    } else {
                        return (data, ACK_D_GENERAL_ERROR);
                    }

                    // Send READ command
                    suart_putc(gpio_base, pin, BL_CMD_READ_FLASH_SIL);
                    suart_putc(gpio_base, pin, read_len as u8);

                    // Read data
                    for _ in 0..read_len {
                        if let Some(byte) = bl_recv_byte(gpio_base, pin) {
                            let _ = data.push(byte);
                        } else {
                            return (data, ACK_D_GENERAL_ERROR);
                        }
                    }

                    // Read and verify CRC (but ignore for now)
                    let _ = bl_recv_byte(gpio_base, pin);
                    let _ = bl_recv_byte(gpio_base, pin);
                }

                (data, ACK_OK)
            }

            CMD_DEVICE_WRITE => {
                if self.current_esc >= ESC_COUNT {
                    return (data, ACK_I_INVALID_CHANNEL);
                }

                if params.is_empty() {
                    return (data, ACK_D_GENERAL_ERROR);
                }

                unsafe {
                    let (gpio_base, pin) = MOTOR_PINS[self.current_esc as usize];

                    // Send SET_ADDRESS command
                    suart_putc(gpio_base, pin, BL_CMD_SET_ADDRESS);
                    suart_putc(gpio_base, pin, 0x00);
                    suart_putc(gpio_base, pin, (addr >> 8) as u8);
                    suart_putc(gpio_base, pin, addr as u8);

                    // Wait for ACK
                    if let Some(ack) = bl_recv_byte(gpio_base, pin) {
                        if ack != BL_CMD_SET_ADDRESS {
                            return (data, ACK_D_GENERAL_ERROR);
                        }
                    } else {
                        return (data, ACK_D_GENERAL_ERROR);
                    }

                    // Send SET_BUFFER command
                    let write_len = params.len();
                    suart_putc(gpio_base, pin, BL_CMD_SET_BUFFER);
                    suart_putc(gpio_base, pin, 0x00);
                    suart_putc(gpio_base, pin, write_len as u8);

                    // Wait for ACK
                    if let Some(ack) = bl_recv_byte(gpio_base, pin) {
                        if ack != BL_CMD_SET_BUFFER {
                            return (data, ACK_D_GENERAL_ERROR);
                        }
                    } else {
                        return (data, ACK_D_GENERAL_ERROR);
                    }

                    // Send data
                    for &byte in params {
                        suart_putc(gpio_base, pin, byte);
                    }

                    // Send CRC
                    let crc = crc16_xmodem(params);
                    suart_putc(gpio_base, pin, (crc >> 8) as u8);
                    suart_putc(gpio_base, pin, crc as u8);

                    // Wait for ACK
                    if let Some(ack) = bl_recv_byte(gpio_base, pin) {
                        if ack != BL_CMD_SET_BUFFER {
                            return (data, ACK_D_GENERAL_ERROR);
                        }
                    } else {
                        return (data, ACK_D_GENERAL_ERROR);
                    }

                    // Send PROG_FLASH command
                    suart_putc(gpio_base, pin, BL_CMD_PROG_FLASH);
                    suart_putc(gpio_base, pin, 0x01);

                    // Wait for ACK
                    if let Some(ack) = bl_recv_byte(gpio_base, pin) {
                        if ack != BL_CMD_PROG_FLASH {
                            return (data, ACK_D_GENERAL_ERROR);
                        }
                    } else {
                        return (data, ACK_D_GENERAL_ERROR);
                    }
                }

                (data, ACK_OK)
            }

            _ => {
                (data, ACK_I_INVALID_CMD)
            }
        }
    }

    fn build_response(&self, cmd: u8, addr: u16, data: &[u8], ack: u8) -> heapless::Vec<u8, 280> {
        let mut response: heapless::Vec<u8, 280> = heapless::Vec::new();

        // Response format: ESC + CMD + ADDR_H + ADDR_L + PARAM_LEN + [PARAM] + ACK + CRC_H + CRC_L
        let _ = response.push(ESC_LOCAL_ESCAPE);
        let _ = response.push(cmd);
        let _ = response.push((addr >> 8) as u8);
        let _ = response.push(addr as u8);
        let _ = response.push(data.len() as u8);

        for &b in data {
            let _ = response.push(b);
        }

        let _ = response.push(ack);

        // Calculate CRC over everything except the CRC itself
        let crc = crc16_xmodem(&response);
        let _ = response.push((crc >> 8) as u8);
        let _ = response.push(crc as u8);

        response
    }
}

/// Initialize motor pins as GPIO inputs with pull-up for passthrough mode
pub fn init_motor_pins_gpio() {
    unsafe {
        for &(gpio_base, pin) in &MOTOR_PINS {
            pin_set_input(gpio_base, pin);
        }
    }
}

/// Check if byte stream contains MSP passthrough command
/// Call this from USB CDC receive path
pub fn check_msp_passthrough(parser: &mut MspParser, byte: u8) -> bool {
    if let Some(cmd) = parser.feed(byte) {
        if cmd == MSP_PASSTHROUGH_ESC_4WAY {
            return true;
        }
    }
    false
}

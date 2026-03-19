use core::ptr::{read_volatile, write_volatile};

// STM32F722 UART4 base address (APB1)
const UART4_BASE: u32 = 0x4000_4C00;

// USART registers (STM32F7 layout)
const UART_CR1: *mut u32 = (UART4_BASE + 0x00) as *mut u32;
const UART_BRR: *mut u32 = (UART4_BASE + 0x0C) as *mut u32;
const UART_ISR: *const u32 = (UART4_BASE + 0x1C) as *const u32;
const UART_TDR: *mut u32 = (UART4_BASE + 0x28) as *mut u32;

// RCC (Reset and Clock Control) for enabling UART4 clock
const RCC_BASE: u32 = 0x4002_3800;
const RCC_APB1ENR: *mut u32 = (RCC_BASE + 0x40) as *mut u32;
const RCC_APB1ENR_UART4EN: u32 = 1 << 19;

// GPIO RCC and configuration for PA0 (UART4 TX)
const RCC_AHB1ENR: *mut u32 = (RCC_BASE + 0x30) as *mut u32;
const RCC_AHB1ENR_GPIOAEN: u32 = 1 << 0;

const GPIOA_BASE: u32 = 0x4002_0000;
const GPIOA_MODER: *mut u32 = (GPIOA_BASE + 0x00) as *mut u32;
const GPIOA_AFRL: *mut u32 = (GPIOA_BASE + 0x20) as *mut u32;

// UART_ISR flags
const UART_ISR_TXE: u32 = 1 << 7;  // Transmit data register empty

// UART_CR1 flags
const UART_CR1_UE: u32 = 1 << 0;   // USART enable
const UART_CR1_TE: u32 = 1 << 3;   // Transmitter enable

#[inline(always)]
fn uart_tx_ready() -> bool {
    unsafe { (read_volatile(UART_ISR) & UART_ISR_TXE) != 0 }
}

#[inline(always)]
fn uart_putc(c: u8) {
    while !uart_tx_ready() {}
    unsafe { write_volatile(UART_TDR, c as u32) };
}

pub fn init() {
    unsafe {
        // Enable GPIOA clock
        let mut rcc_ahb1enr = read_volatile(RCC_AHB1ENR);
        rcc_ahb1enr |= RCC_AHB1ENR_GPIOAEN;
        write_volatile(RCC_AHB1ENR, rcc_ahb1enr);

        // Configure PA0 as alternate function (AF8 = UART4_TX)
        let mut moder = read_volatile(GPIOA_MODER);
        moder = (moder & !(0b11 << 0)) | (0b10 << 0);  // PA0 = alternate function
        write_volatile(GPIOA_MODER, moder);

        let mut afrl = read_volatile(GPIOA_AFRL);
        afrl = (afrl & !(0xF << 0)) | (8 << 0);  // PA0 = AF8
        write_volatile(GPIOA_AFRL, afrl);

        // Enable UART4 clock
        let mut rcc_apb1enr = read_volatile(RCC_APB1ENR);
        rcc_apb1enr |= RCC_APB1ENR_UART4EN;
        write_volatile(RCC_APB1ENR, rcc_apb1enr);

        // Disable UART4 before configuration
        write_volatile(UART_CR1, 0);

        // Configure baud rate for 115200 baud
        // APB1 clock = 54 MHz (216 MHz / 4)
        // BRR = 54000000 / 115200 = 468.75 ≈ 469
        write_volatile(UART_BRR, 469);

        // Enable UART and transmitter
        write_volatile(UART_CR1, UART_CR1_UE | UART_CR1_TE);
    }
}

pub fn write(s: &str) {
    for c in s.bytes() {
        if c == b'\n' {
            uart_putc(b'\r');
        }
        uart_putc(c);
    }
}

pub fn write_decimal(mut x: u32) {
    let mut buf = [0u8; 10];
    let mut i = 0;
    if x == 0 {
        uart_putc(b'0');
        return;
    }
    while x > 0 {
        buf[i] = b'0' + (x % 10) as u8;
        x /= 10;
        i += 1;
    }
    while i > 0 {
        i -= 1;
        uart_putc(buf[i]);
    }
}

use core::ptr::{read_volatile, write_volatile};

const UART0_BASE: u32 = 0x4003_4000;

const UART_DR: *mut u8 = (UART0_BASE + 0x00) as *mut u8;
const UART_FR: *const u8 = (UART0_BASE + 0x18) as *const u8;
const UART_IBRD: *mut u32 = (UART0_BASE + 0x24) as *mut u32;
const UART_FBRD: *mut u32 = (UART0_BASE + 0x28) as *mut u32;
const UART_LCR_H: *mut u32 = (UART0_BASE + 0x2C) as *mut u32;
const UART_CR: *mut u32 = (UART0_BASE + 0x30) as *mut u32;

const UART_FR_TXFF: u8 = 1 << 5;

#[inline(always)]
fn uart_tx_ready() -> bool {
    unsafe { (read_volatile(UART_FR) & UART_FR_TXFF) == 0 }
}

#[inline(always)]
fn uart_putc(c: u8) {
    while !uart_tx_ready() {}
    unsafe { write_volatile(UART_DR, c) };
}

pub fn init() {
    unsafe {
        write_volatile(UART_CR, 0);
        write_volatile(UART_IBRD, 26);
        write_volatile(UART_FBRD, 2);
        write_volatile(UART_LCR_H, (1 << 4) | (1 << 5));
        write_volatile(UART_CR, (1 << 0) | (1 << 8));
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

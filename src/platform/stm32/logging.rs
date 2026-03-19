use core::cell::RefCell;
use core::fmt::Write;
use cortex_m::interrupt::Mutex;
use embassy_stm32::mode::Blocking;
use embassy_stm32::usart::Uart;
use heapless::String;
use log::{LevelFilter, Metadata, Record};
use ringbuffer::{ConstGenericRingBuffer, RingBuffer};

const LOG_RING_SIZE: usize = 4096;
static LOG_RING_UART: Mutex<RefCell<ConstGenericRingBuffer<u8, LOG_RING_SIZE>>> =
    Mutex::new(RefCell::new(ConstGenericRingBuffer::new()));
static LOG_RING_USB: Mutex<RefCell<ConstGenericRingBuffer<u8, LOG_RING_SIZE>>> =
    Mutex::new(RefCell::new(ConstGenericRingBuffer::new()));

pub fn get_uart_ring_buffer() -> &'static Mutex<RefCell<ConstGenericRingBuffer<u8, LOG_RING_SIZE>>> {
    &LOG_RING_UART
}

pub fn get_usb_ring_buffer() -> &'static Mutex<RefCell<ConstGenericRingBuffer<u8, LOG_RING_SIZE>>> {
    &LOG_RING_USB
}

pub struct Logger;

impl log::Log for Logger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= log::Level::Info
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let level_str = match record.level() {
                log::Level::Error => "ERROR",
                log::Level::Warn => "WARN",
                log::Level::Info => "INFO",
                log::Level::Debug => "DEBUG",
                log::Level::Trace => "TRACE",
            };

            let mut buf: String<256> = String::new();
            if writeln!(buf, "[{}] {}\r", level_str, record.args()).is_ok() {
                let data = buf.as_bytes();

                // Write to both ring buffers for UART and USB tasks
                cortex_m::interrupt::free(|cs| {
                    LOG_RING_UART
                        .borrow(cs)
                        .borrow_mut()
                        .extend(data.iter().copied());
                    LOG_RING_USB
                        .borrow(cs)
                        .borrow_mut()
                        .extend(data.iter().copied());
                });
            }
        }
    }

    fn flush(&self) {}
}

static LOGGER: Logger = Logger;

pub fn init_logger() {
    log::set_logger(&LOGGER).unwrap();
    log::set_max_level(LevelFilter::Info);
}

// Task that drains the log ring buffer to UART (blocking mode to avoid DMA conflict with DShot)
#[embassy_executor::task]
pub async fn log_drain_task(mut uart: Uart<'static, Blocking>) -> ! {
    use embassy_time::Timer;

    loop {
        // Collect pending log data
        let mut pending: heapless::Vec<u8, 256> = heapless::Vec::new();

        cortex_m::interrupt::free(|cs| {
            let mut ring = LOG_RING_UART.borrow(cs).borrow_mut();
            while pending.len() < pending.capacity() {
                if let Some(byte) = ring.dequeue() {
                    let _ = pending.push(byte);
                } else {
                    break;
                }
            }
        });

        // Send to UART if we have data (blocking write, no DMA)
        if !pending.is_empty() {
            let _ = uart.blocking_write(&pending);
        }

        Timer::after_millis(10).await;
    }
}

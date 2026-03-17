use core::cell::RefCell;
use core::fmt::Write;
use cortex_m::interrupt::Mutex;
use embassy_rp::uart::BufferedUartTx;
use heapless::String;
use log::{LevelFilter, Metadata, Record};
use ringbuffer::{ConstGenericRingBuffer, RingBuffer};

const LOG_RING_SIZE: usize = 4096;
static LOG_RING: Mutex<RefCell<ConstGenericRingBuffer<u8, LOG_RING_SIZE>>> =
    Mutex::new(RefCell::new(ConstGenericRingBuffer::new()));

static LOG_UART: Mutex<RefCell<Option<BufferedUartTx>>> = Mutex::new(RefCell::new(None));

pub fn get_ring_buffer() -> &'static Mutex<RefCell<ConstGenericRingBuffer<u8, LOG_RING_SIZE>>> {
    &LOG_RING
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

                cortex_m::interrupt::free(|cs| {
                    LOG_RING
                        .borrow(cs)
                        .borrow_mut()
                        .extend(data.iter().copied());
                });

                cortex_m::interrupt::free(|cs| {
                    if let Ok(mut uart_ref) = LOG_UART.borrow(cs).try_borrow_mut() {
                        if let Some(ref mut u) = *uart_ref {
                            let mut written = 0;
                            while written < data.len() {
                                match u.blocking_write(&data[written..]) {
                                    Ok(n) => written += n,
                                    Err(_) => break,
                                }
                            }
                        }
                    }
                });
            }
        }
    }

    fn flush(&self) {}
}

static LOGGER: Logger = Logger;

pub fn init_logger(uart: BufferedUartTx) {
    cortex_m::interrupt::free(|cs| {
        LOG_UART.borrow(cs).replace(Some(uart));
    });
    log::set_logger(&LOGGER).unwrap();
    log::set_max_level(LevelFilter::Info);
}

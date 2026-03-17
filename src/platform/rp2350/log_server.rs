use crate::logging::get_ring_buffer;
use embassy_net::tcp::TcpSocket;
use embassy_net::Stack;
use embassy_time::Timer;
use embedded_io_async::Write;
use log::info;
use ringbuffer::RingBuffer;

const TCP_PORT: u16 = 8082;
const RX_BUFFER_SIZE: usize = 4096;
const TX_BUFFER_SIZE: usize = 4096;

#[embassy_executor::task]
pub async fn log_server_task(stack: &'static Stack<cyw43::NetDriver<'static>>) -> ! {
    info!("Log TCP server starting on port {}", TCP_PORT);

    let mut rx_buffer = [0; RX_BUFFER_SIZE];
    let mut tx_buffer = [0; TX_BUFFER_SIZE];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        
        socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));
        
        let addr_str = if let Some(config) = stack.config_v4() {
            let octets = config.address.address().octets();
            let mut s: heapless::String<32> = heapless::String::new();
            let _ = core::fmt::write(&mut s, format_args!("{}.{}.{}.{}", octets[0], octets[1], octets[2], octets[3]));
            s
        } else {
            heapless::String::from("0.0.0.0")
        };
        info!("Waiting for TCP client on {}:{}", addr_str, TCP_PORT);

        if let Err(e) = socket.accept(TCP_PORT).await {
            info!("Accept error: {:?}", e);
            Timer::after_millis(100).await;
            continue;
        }

        info!("TCP client connected from {:?}", socket.remote_endpoint());

        loop {
            let ring_ref = get_ring_buffer();
            let mut data = [0u8; 4096];
            let len = cortex_m::interrupt::free(|cs| {
                let mut rb = ring_ref.borrow(cs);
                let ring = rb.borrow_mut();
                let avail = ring.len().min(4096);
                for (i, b) in ring.iter().take(avail).enumerate() {
                    data[i] = *b;
                }
                avail
            });

            if len > 0 {
                match socket.write_all(&data[..len]).await {
                    Ok(_) => {}
                    Err(e) => {
                        info!("Write error: {:?}", e);
                        break;
                    }
                }
            }

            Timer::after_millis(50).await;
        }

        info!("TCP client disconnected");
    }
}

pub async fn start_log_server(spawner: &embassy_executor::Spawner, stack: &'static Stack<cyw43::NetDriver<'static>>) {
    spawner.spawn(log_server_task(stack)).unwrap();
}

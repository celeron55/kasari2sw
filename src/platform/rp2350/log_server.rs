use crate::logging::get_ring_buffer;
use embassy_net::tcp::TcpSocket;
use embassy_net::Stack;
use embassy_time::Timer;
use log::info;
use ringbuffer::RingBuffer;

const TCP_PORT: u16 = 8082;
const RX_BUFFER_SIZE: usize = 4096;
const TX_BUFFER_SIZE: usize = 4096;

#[embassy_executor::task]
pub async fn log_server_task(stack: Stack<'static>) -> ! {
    info!("Log TCP server starting on port {}", TCP_PORT);

    let mut rx_buffer = [0; RX_BUFFER_SIZE];
    let mut tx_buffer = [0; TX_BUFFER_SIZE];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        
        socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));
        
        info!("Waiting for TCP client on port {}", TCP_PORT);

        if let Err(e) = socket.accept(TCP_PORT).await {
            info!("Accept error: {:?}", e);
            Timer::after_millis(100).await;
            continue;
        }

        info!("TCP client connected from {:?}", socket.remote_endpoint());

        // Stream logs to client
        loop {
            // Read from ring buffer
            let ring_ref = get_ring_buffer();
            let data = cortex_m::interrupt::free(|cs| {
                let mut rb = ring_ref.borrow(cs);
                let ring = rb.borrow_mut();
                let len = ring.len();
                let mut vec = heapless::Vec::<u8, 4096>::new();
                for b in ring.iter().take(len) {
                    let _ = vec.push(*b);
                }
                vec
            });

            if !data.is_empty() {
                match socket.write(&data).await {
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

pub async fn start_log_server(spawner: &embassy_executor::Spawner, stack: Stack<'static>) {
    spawner.spawn(log_server_task(stack)).unwrap();
}

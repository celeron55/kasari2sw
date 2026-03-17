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
    let mut pending_data: heapless::Vec<u8, 4096> = heapless::Vec::new();

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
            // Pull data from ring buffer if we don't have any pending
            if pending_data.is_empty() {
                cortex_m::interrupt::free(|cs| {
                    let rb = get_ring_buffer().borrow(cs);
                    let mut ring = rb.borrow_mut();
                    while pending_data.capacity() > pending_data.len() {
                        if let Some(byte) = ring.dequeue() {
                            let _ = pending_data.push(byte);
                        } else {
                            break;
                        }
                    }
                });
            }

            // Try to send pending data
            if !pending_data.is_empty() {
                match socket.write(&pending_data).await {
                    Ok(n) => {
                        // Remove sent bytes from the front
                        for i in n..pending_data.len() {
                            pending_data[i - n] = pending_data[i];
                        }
                        unsafe { pending_data.set_len(pending_data.len() - n); }
                    }
                    Err(e) => {
                        info!("Write error: {:?}", e);
                        pending_data.clear();
                        break;
                    }
                }
            }

            Timer::after_millis(50).await;
        }

        info!("TCP client disconnected");
        pending_data.clear();
    }
}

pub async fn start_log_server(spawner: &embassy_executor::Spawner, stack: Stack<'static>) {
    spawner.spawn(log_server_task(stack)).unwrap();
}

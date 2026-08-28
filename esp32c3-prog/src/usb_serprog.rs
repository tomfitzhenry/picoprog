//! Serprog over USB Serial/JTAG.
//!
//! USB Serial/JTAG gives no connect/disconnect signal, so a session is
//! defined as: the first command byte starts it, and a 10 s gap without host
//! data ends it. While a session runs, the USB device and the serprog engine
//! are held exclusively, so serprog-over-TCP waits; between sessions the
//! engine is released and the TCP server can take over.

use embassy_time::Duration;
use esp_hal::usb_serial_jtag::UsbSerialJtag;

use crate::{SerprogMutex, TRANSFER_SIZE};

const USB_IDLE_TIMEOUT: Duration = Duration::from_secs(10);

struct UsbSerialJtagTransport<'a> {
    usb: &'a mut UsbSerialJtag<'static, esp_hal::Async>,
    /// The first command byte, already read while waiting for the session.
    pending: Option<u8>,
}

impl serprog::transport::Transport<TRANSFER_SIZE> for UsbSerialJtagTransport<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()> {
        use embedded_io_async::Read;

        // The first byte of the session was consumed while waiting for it.
        if let Some(b) = self.pending.take() {
            buf[0] = b;
            return Ok(1);
        }

        // A session ends after USB_IDLE_TIMEOUT without host data.
        match embassy_time::with_timeout(USB_IDLE_TIMEOUT, Read::read(&mut *self.usb, buf)).await {
            Ok(result) => result.map_err(|_| ()),
            Err(_) => Err(()), // timed out: session over
        }
    }

    async fn write(&mut self, data: &[u8]) -> Result<(), ()> {
        use embedded_io_async::Write;
        Write::write_all(&mut *self.usb, data)
            .await
            .map_err(|_| ())
    }
}

#[embassy_executor::task]
pub(crate) async fn usb_serprog_task(
    mut usb: UsbSerialJtag<'static, esp_hal::Async>,
    serprog: &'static SerprogMutex,
) {
    loop {
        // Wait for the first byte of a session, polling so the engine (and
        // thus the TCP server) is never held while no host is attached.
        let first = loop {
            use embedded_io_async::Read;
            let mut byte = [0u8; 1];
            match embassy_time::with_timeout(Duration::from_millis(10), Read::read(&mut usb, &mut byte)).await {
                Ok(Ok(1)) => break byte[0],
                _ => {}
            }
        };

        // Session: hold the engine exclusively until the host goes quiet.
        let mut serprog = serprog.lock().await;
        let mut transport = UsbSerialJtagTransport {
            usb: &mut usb,
            pending: Some(first),
        };
        let _ = serprog.run_loop(&mut transport).await;
        // The engine lock drops here; the TCP server can take over.
    }
}

//! Host end-to-end test for the serprog protocol implementation.
//!
//! `rflasher_programs_flash_over_tcp` wires the whole stack together: the
//! project author's real flasher (rflasher) acts as the serprog client over a
//! loopback TCP socket, and the engine drives a stateful flash-chip emulator
//! on the SPI bus. No hardware, no scripted byte expectations.

use core::convert::Infallible;
use std::io::{Read as _, Write as _};
use std::net::{TcpListener, TcpStream};
use std::thread;
use std::time::Duration;
use std::vec::Vec;

use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;

use crate::transport::Transport;
use super::Serprog;

/// Buffer size the transports and `Serprog` instance agree on.
const TRANSFER_SIZE: usize = 64;

const EXPECT_TIMEOUT: Duration = Duration::from_secs(5);

/// `debug!` calls in the firmware need a defmt logger to link against in the
/// test binary; a no-op one is fine here.
#[defmt::global_logger]
struct TestLogger;

unsafe impl defmt::Logger for TestLogger {
    fn acquire() {}

    unsafe fn flush() {}

    unsafe fn release() {}

    unsafe fn write(_bytes: &[u8]) {}
}

/// Poll `fut` to completion, yielding the thread while it is `Pending`.
fn block_on<F: core::future::Future>(fut: F) -> F::Output {
    use core::task::{Context, Poll, Waker};
    let mut fut = std::pin::pin!(fut);
    let waker = Waker::noop();
    let mut cx = Context::from_waker(waker);
    loop {
        if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
            return v;
        }
        std::thread::yield_now();
    }
}

/// Fake GPIO pin: a working stand-in for the host, where there is no real
/// GPIO. Calls succeed and do nothing.
struct FakePin;

impl FakePin {
    fn new() -> Self {
        Self
    }
}

impl embedded_hal::digital::ErrorType for FakePin {
    type Error = Infallible;
}

impl OutputPin for FakePin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Nonblocking TCP transport for the serprog server side, mirroring the
/// ESP32-C3 `TcpTransport`: a read returns whatever bytes are available
/// (partial fills included) and yields to the executor on `WouldBlock`.
struct TcpTransport(TcpStream);

impl TcpTransport {
    fn new(stream: TcpStream) -> Self {
        stream.set_nonblocking(true).unwrap();
        Self(stream)
    }
}

impl Transport<TRANSFER_SIZE> for TcpTransport {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()> {
        loop {
            match self.0.read(buf) {
                Ok(0) => return Err(()), // EOF: client disconnected
                Ok(n) => return Ok(n),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    thread::sleep(Duration::from_millis(1));
                    embassy_futures::yield_now().await;
                }
                Err(_) => return Err(()),
            }
        }
    }

    async fn write(&mut self, data: &[u8]) -> Result<(), ()> {
        let mut offset = 0;
        while offset < data.len() {
            match self.0.write(&data[offset..]) {
                Ok(n) => offset += n,
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    thread::sleep(Duration::from_millis(1));
                    embassy_futures::yield_now().await;
                }
                Err(_) => return Err(()),
            }
        }
        Ok(())
    }
}

// -- the flash-chip emulator -------------------------------------------------

/// Winbond W25Q128FV JEDEC ID (manufacturer, device).
const JEDEC_ID: [u8; 3] = [0xEF, 0x40, 0x18];

/// Which response the next `read` should serve, armed by the last `write`.
enum ReadMode {
    Idle,
    Jedec,
    Status,
    Memory(usize),
}

/// A stateful SPI NOR flash emulator, modelled on rflasher's `DummyFlash`.
///
/// Our serprog engine drives the bus as `write(command bytes)` followed by
/// `read(response)`, so each write decodes the opcode and arms the response
/// the next read will serve.
struct FlashChip {
    data: Vec<u8>,
    status: u8,
    write_enabled: bool,
    read_mode: ReadMode,
    read_cursor: usize,
}

impl FlashChip {
    fn new(size: usize) -> Self {
        Self {
            data: vec![0xFF; size],
            status: 0,
            write_enabled: false,
            read_mode: ReadMode::Idle,
            read_cursor: 0,
        }
    }

    fn addr3(words: &[u8]) -> usize {
        (words[1] as usize) << 16 | (words[2] as usize) << 8 | words[3] as usize
    }
}

impl embedded_hal_async::spi::ErrorType for FlashChip {
    type Error = Infallible;
}

impl SpiBus<u8> for FlashChip {
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for word in words {
            *word = match self.read_mode {
                ReadMode::Jedec => JEDEC_ID.get(self.read_cursor).copied().unwrap_or(0xFF),
                ReadMode::Status => self.status,
                ReadMode::Memory(base) => self
                    .data
                    .get(base + self.read_cursor)
                    .copied()
                    .unwrap_or(0xFF),
                ReadMode::Idle => 0xFF,
            };
            self.read_cursor += 1;
        }
        Ok(())
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.read_cursor = 0;
        match words.first().copied() {
            Some(0x9F) => self.read_mode = ReadMode::Jedec,   // RDID
            Some(0x05) => self.read_mode = ReadMode::Status,  // RDSR
            Some(0x06) => self.write_enabled = true,          // WREN
            Some(0x04) => self.write_enabled = false,         // WRDI
            Some(0x03) | Some(0x0B) => {                      // READ / FAST_READ
                self.read_mode = ReadMode::Memory(FlashChip::addr3(words));
            }
            Some(0x02) if self.write_enabled => {             // PP (page program)
                let addr = FlashChip::addr3(words);
                for (i, byte) in words[4..].iter().enumerate() {
                    self.data[addr + i] &= byte; // flash can only clear bits
                }
                self.write_enabled = false;
                self.read_mode = ReadMode::Idle;
            }
            Some(0x20) if self.write_enabled => {             // SE_20 (sector erase)
                let addr = FlashChip::addr3(words) & !(4096 - 1);
                self.data[addr..addr + 4096].fill(0xFF);
                self.write_enabled = false;
                self.read_mode = ReadMode::Idle;
            }
            Some(0xC7) | Some(0x60) if self.write_enabled => { // CE (chip erase)
                self.data.fill(0xFF);
                self.write_enabled = false;
                self.read_mode = ReadMode::Idle;
            }
            _ => self.read_mode = ReadMode::Idle,
        }
        Ok(())
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.write(write).await?;
        self.read(read).await?;
        Ok(())
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.read(words).await?;
        Ok(())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Blocking TCP transport for the rflasher client, implementing rflasher's
/// (sync-mode) `Transport` trait.
struct RflasherClient(TcpStream);

impl rflasher_programmers::serprog::Transport for RflasherClient {
    fn write(&mut self, data: &[u8]) -> Result<(), rflasher_programmers::serprog::SerprogError> {
        self.0.write_all(data).map_err(rflasher_programmers::serprog::SerprogError::from)
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<(), rflasher_programmers::serprog::SerprogError> {
        self.0.read_exact(buf).map_err(rflasher_programmers::serprog::SerprogError::from)
    }

    fn read_nonblock(
        &mut self,
        buf: &mut [u8],
        timeout_ms: u32,
    ) -> Result<usize, rflasher_programmers::serprog::SerprogError> {
        self.0
            .set_read_timeout(Some(Duration::from_millis(timeout_ms as u64)))
            .map_err(rflasher_programmers::serprog::SerprogError::from)?;
        let result = match self.0.read(buf) {
            Ok(n) => Ok(n),
            Err(e)
                if e.kind() == std::io::ErrorKind::TimedOut
                    || e.kind() == std::io::ErrorKind::WouldBlock =>
            {
                Ok(0)
            }
            Err(e) => Err(e.into()),
        };
        self.0
            .set_read_timeout(Some(EXPECT_TIMEOUT))
            .map_err(rflasher_programmers::serprog::SerprogError::from)?;
        result
    }

    fn write_nonblock(
        &mut self,
        data: &[u8],
        timeout_ms: u32,
    ) -> Result<bool, rflasher_programmers::serprog::SerprogError> {
        self.0
            .set_write_timeout(Some(Duration::from_millis(timeout_ms as u64)))
            .map_err(rflasher_programmers::serprog::SerprogError::from)?;
        let result = match self.0.write_all(data) {
            Ok(()) => Ok(true),
            Err(e)
                if e.kind() == std::io::ErrorKind::TimedOut
                    || e.kind() == std::io::ErrorKind::WouldBlock =>
            {
                Ok(false)
            }
            Err(e) => Err(e.into()),
        };
        self.0
            .set_write_timeout(Some(EXPECT_TIMEOUT))
            .map_err(rflasher_programmers::serprog::SerprogError::from)?;
        result
    }

    fn flush(&mut self) -> Result<(), rflasher_programmers::serprog::SerprogError> {
        Ok(())
    }
}

#[test]
fn rflasher_programs_flash_over_tcp() {
    use rflasher_core::programmer::SpiMaster;
    use rflasher_core::protocol;
    use rflasher_core::spi::{opcodes, SpiCommand};

    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();
    let client = TcpStream::connect(addr).unwrap();
    let (server_stream, _) = listener.accept().unwrap();

    // Server thread: the serprog engine with a flash-chip emulator on its SPI
    // bus, served over a loopback socket until the client disconnects.
    let server_handle = thread::spawn(move || {
        let flash = FlashChip::new(16 * 1024 * 1024);
        let mut serprog = Serprog::new(
            flash,
            FakePin::new(),
            FakePin::new(),
            None::<fn(&mut FlashChip, u32)>,
        );
        let mut transport = TcpTransport::new(server_stream);
        let _ = block_on(serprog.run_loop(&mut transport));
        serprog
    });

    // rflasher drives a real flash session as the client.
    let mut prog = rflasher_programmers::serprog::Serprog::new(RflasherClient(client)).unwrap();

    // Identify the chip.
    let (manufacturer, device) = protocol::read_jedec_id(&mut prog).unwrap();
    assert_eq!((manufacturer, device), (0xEF, 0x4018));

    // Page-program four bytes, then read them back through rflasher.
    let payload = [0x12, 0x34, 0x56, 0x78];
    protocol::write_enable(&mut prog).unwrap();
    let mut cmd = SpiCommand::write_3b(opcodes::PP, 0x1000, &payload);
    prog.execute(&mut cmd).unwrap();
    let mut readback = [0u8; 4];
    let mut cmd = SpiCommand::read_3b(opcodes::READ, 0x1000, &mut readback);
    prog.execute(&mut cmd).unwrap();
    assert_eq!(readback, payload);

    // Sector-erase and confirm the bytes are gone.
    protocol::write_enable(&mut prog).unwrap();
    let mut cmd = SpiCommand::erase_3b(opcodes::SE_20, 0x1000);
    prog.execute(&mut cmd).unwrap();
    let mut readback = [0u8; 4];
    let mut cmd = SpiCommand::read_3b(opcodes::READ, 0x1000, &mut readback);
    prog.execute(&mut cmd).unwrap();
    assert_eq!(readback, [0xFF; 4]);

    // Disconnect; the server's run_loop returns on EOF.
    drop(prog);

    // The emulator's raw memory must reflect what rflasher wrote and erased.
    let serprog = server_handle.join().unwrap();
    let Serprog { spi: flash, .. } = serprog;
    assert_eq!(&flash.data[0x1000..0x1004], &[0xFF; 4]);
}

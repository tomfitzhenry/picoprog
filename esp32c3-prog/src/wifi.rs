//! WiFi + TCP serprog transport.
//!
//! Brings up the station interface, obtains an IPv4 address via DHCP, then
//! serves the serprog protocol over TCP port 4000, one client at a time. The
//! serprog engine (SPI + CS + LED) is shared with the USB serprog task under
//! a mutex: exactly one session runs at a time. If no WiFi credentials were
//! baked in at build time, the firmware is USB-only and this task idles.

use embassy_executor::Spawner;
use embassy_net::{tcp::TcpSocket, Config as NetConfig, Runner, Stack, StackResources};
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_hal::rng::{Trng, TrngSource};
use esp_radio::wifi::{sta::StationConfig, Config, Interface, PowerSaveMode, WifiController};
use rand_core::CryptoRng;
use static_cell::StaticCell;

use crate::{SerprogMutex, TRANSFER_SIZE};

/// Draw a 64-bit seed from a cryptographic RNG. Requiring [`CryptoRng`] here
/// is what keeps the seed from silently falling back to the pseudo-random
/// [`Rng`](esp_hal::rng::Rng).
fn network_seed(mut rng: impl CryptoRng) -> u64 {
    let mut bytes = [0u8; 8];
    rng.fill_bytes(&mut bytes);
    u64::from_le_bytes(bytes)
}

const SSID: &str = env!("PICOPROG_WIFI_SSID");
const PASSWORD: &str = env!("PICOPROG_WIFI_PASSWORD");

const LISTEN_PORT: u16 = 4000;
const SOCKET_RX_SIZE: usize = 4096;
const SOCKET_TX_SIZE: usize = 4096;
const IDLE_TIMEOUT: Duration = Duration::from_secs(60);

// Two sockets: DHCP and the serprog TCP server.
static STACK_RESOURCES: StaticCell<StackResources<2>> = StaticCell::new();
static SOCKET_RX_BUFFER: StaticCell<[u8; SOCKET_RX_SIZE]> = StaticCell::new();
static SOCKET_TX_BUFFER: StaticCell<[u8; SOCKET_TX_SIZE]> = StaticCell::new();

pub async fn serve(
    spawner: Spawner,
    wifi: esp_hal::peripherals::WIFI<'static>,
    rng: esp_hal::peripherals::RNG<'static>,
    mut adc1: esp_hal::peripherals::ADC1<'static>,
    serprog: &'static SerprogMutex,
) -> ! {
    if SSID.is_empty() || PASSWORD.is_empty() {
        // USB-only mode: no WiFi credentials were baked in at build time.
        loop {
            core::future::pending::<()>().await
        }
    }

    // esp-radio needs a dynamic allocator: register the 72 KiB heap region
    // before any allocation below.
    esp_alloc::heap_allocator!(size: 72 * 1024);

    // The network stack's seed (TCP ISNs, ephemeral ports, DHCP xid) must be
    // unpredictable. `Trng` is the ESP32-C3's true random number generator; it
    // satisfies the `CryptoRng` bound `network_seed` requires.
    let seed = {
        let _source = TrngSource::new(rng, adc1.reborrow());
        let trng = Trng::try_new().expect("TRNG source should be active");
        network_seed(trng)
    };

    let (mut controller, interfaces) = esp_radio::wifi::new(wifi, Default::default()).unwrap();
    let wifi_interface = interfaces.station;
    controller.set_power_saving(PowerSaveMode::None).unwrap();

    let net_config = NetConfig::dhcpv4(Default::default());
    let (stack, runner) = embassy_net::new(
        wifi_interface,
        net_config,
        STACK_RESOURCES.init(StackResources::<2>::new()),
        seed,
    );

    spawner.spawn(connection_task(controller).unwrap());
    spawner.spawn(net_task(runner).unwrap());

    // Only accept clients once the link is up and DHCP has assigned an IP.
    while !stack.is_link_up() {
        Timer::after(Duration::from_millis(500)).await;
    }
    while stack.config_v4().is_none() {
        Timer::after(Duration::from_millis(500)).await;
    }

    // Reusable socket buffers; each session reborrows them for a fresh socket.
    let rx_buf: &'static mut [u8] = SOCKET_RX_BUFFER.init([0u8; SOCKET_RX_SIZE]);
    let tx_buf: &'static mut [u8] = SOCKET_TX_BUFFER.init([0u8; SOCKET_TX_SIZE]);

    loop {
        // A fresh TcpSocket per session: a socket left over from a finished
        // session is not in the Closed state, so smoltcp's listen() would
        // reject it with InvalidState and the server would never re-listen.
        let mut transport = TcpTransport::new(stack, &mut *rx_buf, &mut *tx_buf);
        transport.accept().await;
        // One client at a time; the engine is shared with USB serprog, so a
        // USB session running elsewhere makes this wait.
        let mut serprog = serprog.lock().await;
        let _ = serprog.run_loop(&mut transport).await;
        drop(serprog);
        transport.abort().await;
    }
}

#[embassy_executor::task]
async fn connection_task(mut controller: WifiController<'static>) {
    let station_config = Config::Station(
        StationConfig::default()
            .with_ssid(SSID)
            .with_password(PASSWORD.into()),
    );
    controller.set_config(&station_config).unwrap();

    loop {
        if !controller.is_connected() {
            match controller.connect_async().await {
                Ok(_) => continue,
                Err(_e) => {
                    Timer::after(Duration::from_millis(5000)).await;
                    continue;
                }
            }
        }
        // Connected: wait until the AP drops us, then reconnect.
        let _ = controller.wait_for_disconnect_async().await;
        Timer::after(Duration::from_millis(5000)).await;
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface<'static>>) {
    runner.run().await
}

struct TcpTransport<'a> {
    socket: TcpSocket<'a>,
}

impl<'a> TcpTransport<'a> {
    fn new(stack: Stack<'static>, rx: &'a mut [u8], tx: &'a mut [u8]) -> Self {
        let mut socket = TcpSocket::new(stack, rx, tx);
        socket.set_timeout(Some(IDLE_TIMEOUT));
        Self { socket }
    }

    /// Accept one client; retry on error so the server never exits.
    async fn accept(&mut self) {
        loop {
            match self.socket.accept(LISTEN_PORT).await {
                Ok(_) => return,
                Err(_e) => {
                    Timer::after(Duration::from_millis(500)).await;
                }
            }
        }
    }

    /// Abort the connection and flush the RST out before the socket is
    /// dropped, following the embassy-net server example. `close()` is not
    /// used: it leaves the socket outside the Closed state, which would make
    /// the next session's listen() fail with InvalidState.
    async fn abort(&mut self) {
        self.socket.abort();
        let _ = self.socket.flush().await;
    }
}

impl serprog::transport::Transport<TRANSFER_SIZE> for TcpTransport<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()> {
        // A single read: TCP delivers partial fills (as soon as >= 1 byte is
        // available), and the caller accounts for the actual byte count
        // returned. A timeout bounds a client that connects then stalls
        // mid-session, so the port re-opens instead of blocking forever.
        match embassy_time::with_timeout(IDLE_TIMEOUT, self.socket.read(buf)).await {
            Err(_) => Err(()),    // session timeout: client went silent
            Ok(Ok(0)) => Err(()), // EOF: client disconnected
            Ok(Ok(n)) => Ok(n),
            Ok(Err(_)) => Err(()),
        }
    }

    async fn write(&mut self, data: &[u8]) -> Result<(), ()> {
        use embedded_io_async_06::Write as _;
        // Bound the write too: a peer that stops reading would otherwise hold
        // the session forever once the socket's send buffer fills.
        embassy_time::with_timeout(IDLE_TIMEOUT, self.socket.write_all(data))
            .await
            .map_err(|_| ())?
            .map_err(|_| ())
    }
}

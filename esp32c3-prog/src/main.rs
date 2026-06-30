#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

esp_bootloader_esp_idf::esp_app_desc!();

use esp_backtrace as _;
use esp_hal::{
    gpio::{Level, Output, OutputConfig},
    spi::master::{Config as SpiConfig, Spi},
    time::Rate,
    timer::timg::TimerGroup,
    usb_serial_jtag::UsbSerialJtag,
};

const USB_BUFFER_SIZE: usize = 64;

struct UsbSerialJtagTransport<'d> {
    usb_serial: UsbSerialJtag<'d, esp_hal::Async>,
}

impl<'d> serprog::transport::Transport<USB_BUFFER_SIZE> for UsbSerialJtagTransport<'d> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<(), ()> {
        use embedded_io_async::Read;
        Read::read(&mut self.usb_serial, buf)
            .await
            .map_err(|_| ())?;
        Ok(())
    }

    async fn write(&mut self, data: &[u8]) -> Result<(), ()> {
        use embedded_io_async::Write;
        Write::write_all(&mut self.usb_serial, data)
            .await
            .map_err(|_| ())
    }
}

#[esp_rtos::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let sw_int =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // Direct IOMUX pinout, per the ESP32-C3 datasheet.
    let sclk = peripherals.GPIO6;
    let mosi = peripherals.GPIO2;
    let miso = peripherals.GPIO4;
    let cs = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    let led = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());

    // SPI0/1 are reserved for flash/PSRAM, per the ESP32-C3 datasheet.
    let spi = Spi::new(peripherals.SPI2, SpiConfig::default())
        .expect("SPI2 init failed")
        .with_sck(sclk)
        .with_mosi(mosi)
        .with_miso(miso)
        .into_async();

    let usb_serial = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();
    let transport = UsbSerialJtagTransport { usb_serial };

    let set_freq_cb = move |spi: &mut Spi<'_, esp_hal::Async>, freq: u32| {
        let config = SpiConfig::default().with_frequency(Rate::from_hz(freq));
        // Errors are intentionally dropped: any output channel collides with
        // the serprog protocol on USB Serial/JTAG.
        let _ = spi.apply_config(&config);
    };

    let serprog = serprog::Serprog::<_, _, _, _, _, USB_BUFFER_SIZE>::new(
        spi,
        cs,
        led,
        transport,
        Some(set_freq_cb),
    );

    serprog.run_loop().await
}

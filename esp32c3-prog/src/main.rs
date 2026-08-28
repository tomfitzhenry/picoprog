#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

esp_bootloader_esp_idf::esp_app_desc!();

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use esp_backtrace as _;
use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{Level, Output, OutputConfig},
    spi::master::{Config as SpiConfig, Spi, SpiDmaBus},
    time::Rate,
    timer::timg::TimerGroup,
    usb_serial_jtag::UsbSerialJtag,
};
use static_cell::StaticCell;

mod usb_serprog;
mod wifi;

/// Serprog transfer size. 64 keeps the serprog run loop's channel buffers
/// small enough for embassy task stacks, on both transports.
pub(crate) const TRANSFER_SIZE: usize = 64;

/// The ESP32-C3's async `SpiBus` is only implemented for DMA-backed SPI; the
/// interrupt-based `Spi<Async>` never completes reads.
pub(crate) type SpiBusType = SpiDmaBus<'static, esp_hal::Async>;

fn set_freq_cb(spi: &mut SpiBusType, freq: u32) {
    let config = SpiConfig::default().with_frequency(Rate::from_hz(freq));
    // Errors are intentionally dropped: any output channel collides with
    // the serprog protocol.
    let _ = spi.apply_config(&config);
}

/// The serprog engine (SPI + CS + LED). The USB and TCP session tasks take
/// turns holding it under this mutex: exactly one serprog session runs at a
/// time, the other transport's task waits.
pub(crate) type SerprogEngine =
    serprog::Serprog<SpiBusType, Output<'static>, Output<'static>, fn(&mut SpiBusType, u32), TRANSFER_SIZE>;
pub(crate) type SerprogMutex = Mutex<CriticalSectionRawMutex, SerprogEngine>;

#[esp_rtos::main]
async fn main(spawner: embassy_executor::Spawner) {
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
    //
    // DMA-backed SPI: the interrupt-based `Spi<Async>` does not implement the
    // async SpiBus read path correctly on ESP32-C3, so a DMA channel and DMA
    // buffers are required for serprog reads to complete.
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4096);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
    let spi: SpiBusType = Spi::new(peripherals.SPI2, SpiConfig::default())
        .expect("SPI2 init failed")
        .with_sck(sclk)
        .with_mosi(mosi)
        .with_miso(miso)
        .with_dma(peripherals.DMA_CH0)
        .with_buffers(dma_rx_buf, dma_tx_buf)
        .into_async();

    // Type-erase the callback so the serprog engine's full type can be named.
    let freq_callback: fn(&mut SpiBusType, u32) = set_freq_cb;
    let serprog = serprog::Serprog::<_, _, _, _, TRANSFER_SIZE>::new(spi, cs, led, Some(freq_callback));

    static SERPROG: StaticCell<SerprogMutex> = StaticCell::new();
    let serprog = SERPROG.init(Mutex::new(serprog));

    let usb_serial = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();

    // One firmware serves both transports. The serprog engine is shared via
    // the mutex; a session on either transport holds it until it ends.
    spawner.spawn(usb_serprog::usb_serprog_task(usb_serial, serprog).unwrap());
    wifi::serve(spawner, peripherals.WIFI, peripherals.RNG, peripherals.ADC1, serprog).await
}

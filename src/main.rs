//! This example shows how to use USB (Universal Serial Bus) in the RP2040 chip as well as how to create multiple usb classes for one device
//!
//! This creates a USB serial port that echos. It will also print out logging information on a separate serial device

#![no_std]
#![no_main]

use core::cell::RefCell;

use arducam_legacy::Arducam;
use cyw43::Control;
use cyw43_pio::PioSpi;
use defmt::{info, panic};
use embassy_executor::Spawner;
use embassy_rp::{bind_interrupts, spi};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0, USB};
use embassy_rp::pio::Pio;
use embassy_rp::usb::{Driver, Instance};
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_time::{Delay, Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

type UsbInterruptHandler<T> = embassy_rp::usb::InterruptHandler<T>;
type PIOInterruptHandler<T> = embassy_rp::pio::InterruptHandler<T>;

// Create embassy-usb DeviceBuilder using the driver and config.
// It needs some buffers for building the descriptors.
static mut CONFIG_DESCRIPTOR: [u8; 256] = [0; 256];
static mut BOS_DESCRIPTOR: [u8; 256] = [0; 256];
static mut CONTROL_BUF: [u8; 64] = [0; 64];

bind_interrupts!(struct USBIrqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

bind_interrupts!(struct PIOIrqs {
    PIO0_IRQ_0 => PIOInterruptHandler<PIO0>;
});


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello there!");

    let p = embassy_rp::init(Default::default());

    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, USBIrqs);

    // Create embassy-usb Config
    let config = create_usb_config();


    // static USB_STATE: StaticCell<State> = StaticCell::new();
    static LOGGER_STATE: StaticCell<State> = StaticCell::new();

    // let state = USB_STATE.init(State::new());
    let logger_state = LOGGER_STATE.init(State::new());


    #[allow(static_mut_refs)]
    let mut builder = unsafe {
        Builder::new(
            driver,
            config,
            &mut CONFIG_DESCRIPTOR,
            &mut BOS_DESCRIPTOR,
            &mut [], // no msos descriptors
            &mut CONTROL_BUF
        )
    };


    // Create a class for the logger
    let logger_class = CdcAcmClass::new(&mut builder, logger_state, 64);

    // Build the builder.
    let usb = builder.build();

    // Setup wifi chip
    let fw = include_bytes!("..\\43439A0.bin");
    let clm = include_bytes!("..\\43439A0_clm.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, PIOIrqs);
    let net_spi = PioSpi::new(&mut pio.common, pio.sm0, pio.irq0, cs, p.PIN_24, p.PIN_29, p.DMA_CH0);

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_net_device, mut wifi_chip, runner) = cyw43::new(state, pwr, net_spi, fw).await;
    spawner.spawn(cyw43_task(runner)).unwrap();

    wifi_chip.init(clm).await;
    wifi_chip
    .set_power_management(cyw43::PowerManagementMode::PowerSave)
    .await;

    // Example pinout configuration
    // Adapt to your HAL crate
    // let _arducam_spi_mosi = Pin::new(Port::D, 4, PinMode::Alt(5));
    // let _arducam_spi_miso = Pin::new(Port::D, 3, PinMode::Alt(5));
    // let _arducam_spi_sck = Pin::new(Port::D, 1, PinMode::Alt(5));
    let arducam_cs = Output::new(p.PIN_0, Level::High);
    static SPI_BUS: StaticCell<NoopMutex<RefCell<spi::Spi<'_, embassy_rp::peripherals::SPI0, spi::Blocking>>>> = StaticCell::new();
    let arducam_spi = embassy_rp::spi::Spi::new_blocking(p.SPI0, p.PIN_2, p.PIN_3, p.PIN_4, embassy_rp::spi::Config::default());
    let arducam_spi_bus = NoopMutex::new(RefCell::new(arducam_spi));
    let arducam_spi_bus = SPI_BUS.init(arducam_spi_bus);
    let arducam_spi_device = embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice::new(arducam_spi_bus, arducam_cs);

    // let arducam_spi = todo!(); //PioSpi::new(&mut pio.common, pio.sm1, pio.irq0, arducam_cs, p.PIN_4, p.PIN_1, p.DMA_CH0);
    // let mut arducam_i2c_sda = Output::new(p.PIN_8, Level::Low);
    // arducam_i2c_sda.output_type(OutputType::OpenDrain);
    // let mut arducam_i2c_scl = Output::new(p.PIN_9, Level::Low);
    // arducam_i2c_scl.output_type(OutputType::OpenDrain);
    let arducam_i2c = embassy_rp::i2c::I2c::new_blocking(p.I2C0, p.PIN_9, p.PIN_8, embassy_rp::i2c::Config::default());

    let arducam = Arducam::new(
        arducam_spi_device,
        arducam_i2c,
        arducam_legacy::Resolution::Res320x240, arducam_legacy::ImageFormat::JPEG
        );


    spawner.spawn(blinky_task(wifi_chip)).unwrap();
    spawner.spawn(usb_management_task(usb)).unwrap();
    spawner.spawn(logger_task(logger_class)).unwrap();
    spawner.spawn(doot_camera(arducam)).unwrap();

}

fn create_usb_config<'a>() -> Config<'a> {
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-serial example");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    config
}

#[embassy_executor::task]
async fn doot_camera(mut arducam: Arducam<embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice<'static, embassy_sync::blocking_mutex::raw::NoopRawMutex, spi::Spi<'static, embassy_rp::peripherals::SPI0, spi::Blocking>, Output<'static>>, embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Blocking>>) {
    let delay = Duration::from_millis(100);
    let mut raw_delay = Delay{};

    Timer::after(delay).await;

    log::info!("StartInit");
    arducam.init(&mut raw_delay).unwrap();
    arducam.set_resolution(arducam_legacy::Resolution::Res320x240).unwrap();
    log::info!("CheckConnection");
    let connected = arducam.is_connected().unwrap();
    log::info!("Connected: {}", connected);
    // loop {
        log::info!("StartCapture");
        arducam.start_capture().unwrap();
        log::info!("AwaitCaptureDone");
        while !arducam.is_capture_done().unwrap() { Timer::after(delay).await; }
        log::info!("CaptureDone");
        let mut image = [0_u8; 8192];
        let length = arducam.get_fifo_length().unwrap();
        let final_length = arducam.read_captured_image(&mut image).unwrap();
        log::info!("FifoLength: {}", length);
        log::info!("FinalImageLength: {}", final_length);
        // log::info!("Image: {:?}", image);
        Timer::after(delay).await;
        Timer::after(delay).await;
        Timer::after(delay).await;
        Timer::after(delay).await;
    // }
}

#[embassy_executor::task]
async fn usb_management_task(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) {
    usb.run().await
}

#[embassy_executor::task]
async fn logger_task(logger_class: CdcAcmClass<'static, Driver<'static, USB>>) {
    embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, logger_class).await
}

#[embassy_executor::task]
async fn blinky_task(mut control: Control<'static>) {
    let delay = Duration::from_secs(1);
    loop {
            // log::info!("led on!");
            control.gpio_set(0, true).await;
            Timer::after(delay).await;

            // log::info!("led off!");
            control.gpio_set(0, false).await;
            Timer::after(delay).await;
        }
}

#[embassy_executor::task]
async fn echo_task(mut class: CdcAcmClass<'static, Driver<'static, USB>>) {
    loop {
        class.wait_connection().await;
        log::info!("Connected");
        let _ = echo(&mut class).await;
        log::info!("Disconnected");
    }
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn echo<'d, T: Instance + 'd>(class: &mut CdcAcmClass<'d, Driver<'d, T>>) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("data: {:x}", data);
        class.write_packet(data).await?;
    }
}

#[embassy_executor::task]
async fn cyw43_task(runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>) -> ! {
    runner.run().await
}

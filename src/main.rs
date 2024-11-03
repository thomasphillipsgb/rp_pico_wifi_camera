//! This example shows how to use USB (Universal Serial Bus) in the RP2040 chip as well as how to create multiple usb classes for one device
//!
//! This creates a USB serial port that echos. It will also print out logging information on a separate serial device

#![no_std]
#![no_main]

use core::cell::RefCell;
use core::slice;
use core::str::from_utf8;

use arducam_legacy::Arducam;
use cyw43::{Control, NetDriver};
use cyw43_pio::PioSpi;
use defmt::{info, panic};
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Config as NetConfig, Stack, StackResources};
use embassy_rp::rtc::DayOfWeek;
use embassy_rp::{bind_interrupts, spi};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0, USB};
use embassy_rp::pio::Pio;
use embassy_rp::usb::{Driver, Instance};
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_time::{Delay, Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder as UsbBuilder;
use embassy_usb::Config as UsbConfig;
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
    let p = embassy_rp::init(Default::default());

    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, USBIrqs);
    let config = create_usb_config();
    static LOGGER_STATE: StaticCell<State> = StaticCell::new();
    let logger_state = LOGGER_STATE.init(State::new());


    #[allow(static_mut_refs)]
    let mut builder = unsafe {
        UsbBuilder::new(
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

    spawner.spawn(usb_management_task(usb)).unwrap();
    spawner.spawn(logger_task(logger_class)).unwrap();



    // Setup wifi chip
    let fw = include_bytes!("..\\43439A0.bin");
    let clm = include_bytes!("..\\43439A0_clm.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, PIOIrqs);
    let net_spi = PioSpi::new(&mut pio.common, pio.sm0, pio.irq0, cs, p.PIN_24, p.PIN_29, p.DMA_CH0);

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, net_spi, fw).await;
    spawner.spawn(cyw43_task(runner)).unwrap();

    control.init(clm).await;
    control.set_power_management(cyw43::PowerManagementMode::PowerSave).await;


// Setup TCP server
    let config = NetConfig::dhcpv4(Default::default());
    //let config = embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
    //    address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 69, 2), 24),
    //    dns_servers: Vec::new(),
    //    gateway: Some(Ipv4Address::new(192, 168, 69, 1)),
    //});

    // Generate random seed
    // let seed = rng.next_u64();

    // Init network stack
    static RESOURCES: StaticCell<embassy_net::StackResources<2>> = StaticCell::new();
    static STACK: StaticCell<Stack<NetDriver>> = StaticCell::new();

    let stack = embassy_net::Stack::new(
        net_device,
        config,
        RESOURCES.init(embassy_net::StackResources::new()),
        1234522
    );
    let stack = &*STACK.init(stack);
    // Launch network task that runs `stack.run().await`
    spawner.spawn(net_task(stack)).unwrap();

    loop {
        match control.join_wpa2("Phillips_2.4", "ThomasHouse@63!").await
        {
            Ok(_) => {
                log::info!("join success");

                break
            },
            Err(err) => {
                log::info!("join failed with status={}", err.status);
            }
        }
    }

    // Wait for DHCP config
    stack.wait_config_up().await;



    let arducam_cs = Output::new(p.PIN_5, Level::High);
    static SPI_BUS: StaticCell<NoopMutex<RefCell<spi::Spi<'_, embassy_rp::peripherals::SPI0, spi::Blocking>>>> = StaticCell::new();
    let mut config = embassy_rp::spi::Config::default();
    config.phase = spi::Phase::CaptureOnFirstTransition;
    config.polarity = spi::Polarity::IdleLow;
    config.frequency = 8_000_000;

    let arducam_spi = embassy_rp::spi::Spi::new_blocking(p.SPI0, p.PIN_2, p.PIN_3, p.PIN_4, config);
    let arducam_spi_bus = NoopMutex::new(RefCell::new(arducam_spi));
    let arducam_spi_bus = SPI_BUS.init(arducam_spi_bus);
    let arducam_spi_device = embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice::new(arducam_spi_bus, arducam_cs);
    let arducam_i2c = embassy_rp::i2c::I2c::new_blocking(p.I2C0, p.PIN_9, p.PIN_8, embassy_rp::i2c::Config::default());

    let arducam = Arducam::new(
        arducam_spi_device,
        arducam_i2c,
        arducam_legacy::Resolution::Res176x144, arducam_legacy::ImageFormat::JPEG
        );

    spawner.spawn(net_stack(stack, control, arducam)).unwrap();

}

fn create_usb_config<'a>() -> UsbConfig<'a> {
    let mut config = UsbConfig::new(0xc0de, 0xcafe);
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
async fn doot_camera() {

}

#[embassy_executor::task]
async fn usb_management_task(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) {
    usb.run().await
}

#[embassy_executor::task]
async fn net_task(runner: &'static Stack<NetDriver<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_stack(stack: &'static Stack<NetDriver<'static>>, mut control: cyw43::Control<'static>, mut arducam: Arducam<embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice<'static, embassy_sync::blocking_mutex::raw::NoopRawMutex, spi::Spi<'static, embassy_rp::peripherals::SPI0, spi::Blocking>, Output<'static>>, embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Blocking>>) -> ! {
    let mut rx_buffer = [0; 10];
    let mut tx_buffer = [0; 8192];
    let delay = Duration::from_secs(8);

    let camera_query_delay = Duration::from_millis(100);
    let transfer_delay = Duration::from_millis(600);
    let mut raw_delay = Delay{};

    Timer::after(camera_query_delay).await;

    log::info!("StartInit");
    arducam.init(&mut raw_delay).unwrap();

    let chip_info = arducam.get_sensor_chipid().unwrap();
    log::info!("ChipId: {:?}", chip_info);
    arducam.set_resolution(arducam_legacy::Resolution::Res320x240).unwrap();
    log::info!("CheckConnection");
    let connected = arducam.is_connected().unwrap();
    log::info!("Connected: {}", connected);

    const HOLDING: [u8; 8192] = [0_u8; 8192];
    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));

        control.gpio_set(0, false).await;
        log::info!("Listening on TCP:1234...");
        if let Err(e) = socket.accept(1234).await {
            log::info!("accept error: {:?}", e);
            continue;
        }

        log::info!("Received connection from {:?}", socket.remote_endpoint());
        control.gpio_set(0, true).await;
        // log::info!("StartCapture");

        loop {
            arducam.start_capture().unwrap();

            while !arducam.is_capture_done().unwrap() { Timer::after(camera_query_delay).await; }
            let mut image = [0_u8; 8192];
            let length = arducam.get_fifo_length().unwrap();

            let final_length = arducam.read_captured_image(&mut image).unwrap();
            log::info!("FifoLength: {}", length);
            log::info!("FinalImageLength: {}", final_length);

            Timer::after(transfer_delay).await;

            let range = 0..=length as usize;
            log::info!("Range: {:?}", range);

            match socket.write(&image[range]).await {
                Ok(size) => {
                    log::info!("written: {:?}", size);
                }
                Err(e) => {
                    log::info!("write error: {:?}", e);
                    break;
                }
            };

            log::info!("FinishWrite");
        }

    }
}

#[embassy_executor::task]
async fn logger_task(logger_class: CdcAcmClass<'static, Driver<'static, USB>>) {
    embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, logger_class).await
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

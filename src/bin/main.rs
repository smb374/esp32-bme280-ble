#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::fmt::Write;

use bme280_sensor::bme280::{BME280, PresUnit, TempUnit};
use bt_hci::controller::ExternalController;
use defmt::Debug2Format;
use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::Timer;
use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, iso_8859_1::FONT_9X15},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};
use embedded_hal_bus::{spi::AtomicDevice, util::AtomicCell};
use esp_hal::{
    Blocking,
    clock::CpuClock,
    delay::Delay,
    gpio::{self, Level, Output},
    ledc::{self, Ledc, LowSpeed, channel::ChannelIFace, timer::TimerIFace},
    rng::{Trng, TrngSource},
    spi,
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_radio::ble::controller::BleConnector;
use mipidsi::{
    interface::SpiInterface,
    models::ST7789,
    options::{ColorInversion, ColorOrder, Orientation, Rotation},
};
use trouble_host::prelude::*;
use {esp_backtrace as _, esp_println as _};

extern crate alloc;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 3;
const NUS_BUF_SIZE: usize = 256;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

type BLECtrl<'a, const N: usize> = ExternalController<BleConnector<'a>, N>;
type SPIDevice<'a> = AtomicDevice<'a, spi::master::Spi<'a, Blocking>, Output<'a>, Delay>;

#[gatt_server]
struct Server {
    nus: NordicUartService,
}

#[gatt_service(uuid = "6e400001-b5a3-f393-e0a9-e50e24dcca9e")]
struct NordicUartService {
    /// TX Characteristic (ESP32 transmits, central receives via notify)
    #[characteristic(uuid = "6e400003-b5a3-f393-e0a9-e50e24dcca9e", read, notify)]
    tx: heapless::Vec<u8, NUS_BUF_SIZE>,

    /// RX Characteristic (ESP32 receives, central writes)
    #[characteristic(
        uuid = "6e400002-b5a3-f393-e0a9-e50e24dcca9e",
        write,
        write_without_response
    )]
    rx: heapless::Vec<u8, NUS_BUF_SIZE>,
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.1.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);
    // COEX needs more RAM - so we've added some more
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    defmt::info!("Embassy initialized!");
    let _trng_source = TrngSource::new(peripherals.RNG, peripherals.ADC1);
    let trng: &'static mut Trng = mk_static!(Trng, Trng::try_new().expect("No TRNG available")); // Ok when there's a TrngSource accessible

    let radio_init: &'static esp_radio::Controller<'static> = mk_static!(
        esp_radio::Controller<'static>,
        esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller")
    );
    let (mut _wifi_controller, interfaces) =
        esp_radio::wifi::new(radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    let address: Address = Address::random(interfaces.sta.mac_address());
    let raddr = address.to_bytes();
    defmt::info!(
        "Address is {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        raddr[1],
        raddr[2],
        raddr[3],
        raddr[4],
        raddr[5],
        raddr[6],
    );
    // find more examples https://github.com/embassy-rs/trouble/tree/main/examples/esp32
    let transport: BleConnector<'static> =
        BleConnector::new(radio_init, peripherals.BT, Default::default()).unwrap();
    let ble_controller = ExternalController::<BleConnector<'static>, 20>::new(transport);
    let resources: &'static mut HostResources<_, _, _> = mk_static!(
        HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX>,
        HostResources::new()
    );
    let stack = mk_static!(
        Stack<'static, BLECtrl<'static, 20>, DefaultPacketPool>,
        trouble_host::new(ble_controller, resources)
            .set_random_address(address)
            .set_random_generator_seed(trng)
    );

    let Host {
        peripheral, runner, ..
    }: Host<'static, BLECtrl<'static, 20>, DefaultPacketPool> = stack.build();

    // TODO: Spawn some tasks
    spawner.must_spawn(ble_task(runner));

    // BME280 Pins
    // VCC
    // GND
    // SCK
    // MOSI/SDA/SDI
    // CS/CSB
    // MISO/SDO
    let spi_cfg = spi::master::Config::default()
        .with_mode(spi::Mode::_0)
        .with_frequency(Rate::from_mhz(20));
    let mspi = mk_static!(
        AtomicCell<spi::master::Spi<'static, Blocking>>,
        AtomicCell::new(
            spi::master::Spi::new(peripherals.SPI2, spi_cfg)
                .expect("Failed to create master SPI bus")
                .with_sck(peripherals.GPIO6)
                .with_mosi(peripherals.GPIO7)
                .with_miso(peripherals.GPIO2)
        )
    );

    let cs0 = Output::new(peripherals.GPIO4, Level::High, Default::default());
    let cs1 = Output::new(peripherals.GPIO5, Level::High, Default::default());
    let dc = Output::new(peripherals.GPIO3, Level::Low, Default::default());
    let rst = Output::new(peripherals.GPIO10, Level::Low, Default::default());

    let sensor_spi = AtomicDevice::new(mspi, cs0, Delay::new()).unwrap();

    let sensor = mk_static!(
        Mutex<CriticalSectionRawMutex, BME280<SPIDevice<'static>, Delay>>,
       Mutex::new(BME280::new(sensor_spi, Delay::new(), Default::default()))
    );

    sensor.get_mut().init().expect("Failed to init BME280");

    spawner.must_spawn(advertise_loop(peripheral, sensor));

    let lcd_spi = AtomicDevice::new(mspi, cs1, Delay::new()).unwrap();
    let lcd_di = SpiInterface::new(lcd_spi, dc, mk_static!([u8; 4096], [0u8; 4096]));

    let mut lcd = mipidsi::Builder::new(ST7789, lcd_di)
        .reset_pin(rst)
        .color_order(ColorOrder::Rgb)
        .display_size(240, 320)
        .orientation(Orientation::new().rotate(Rotation::Deg90))
        .invert_colors(ColorInversion::Inverted)
        .init(&mut Delay::new())
        .expect("Failed to init display");

    {
        let mut ledc = Ledc::new(peripherals.LEDC);
        ledc.set_global_slow_clock(ledc::LSGlobalClkSource::APBClk);

        let mut lstimer0: ledc::timer::Timer<'_, LowSpeed> =
            ledc.timer::<LowSpeed>(ledc::timer::Number::Timer0);
        lstimer0
            .configure(ledc::timer::config::Config {
                duty: ledc::timer::config::Duty::Duty13Bit,
                clock_source: ledc::timer::LSClockSource::APBClk,
                frequency: Rate::from_hz(5000),
            })
            .expect("Failed to configure backlight timer");

        let mut chan: ledc::channel::Channel<'_, LowSpeed> =
            ledc.channel::<LowSpeed>(ledc::channel::Number::Channel0, peripherals.GPIO11);
        chan.configure(ledc::channel::config::Config {
            timer: &lstimer0,
            duty_pct: 100,
            drive_mode: gpio::DriveMode::PushPull,
        })
        .expect("Failed to configure backlight channel");

        chan.set_duty(25).expect("Failed to update backlight");
    }

    lcd.clear(Rgb565::BLACK).expect("Failed to clear screen");
    Timer::after_millis(10).await;

    let txt_style = MonoTextStyleBuilder::new()
        .background_color(Rgb565::BLACK)
        .text_color(Rgb565::WHITE)
        .font(&FONT_9X15)
        .build();

    let mut txt_buf: heapless::String<256> = heapless::String::new();

    loop {
        let mut guard = sensor.lock().await;
        let res = guard.read_all(TempUnit::Celsius, PresUnit::HPa);
        drop(guard);
        match res {
            Ok(m) => {
                txt_buf.clear();
                write!(
                    &mut txt_buf,
                    "{: <8}: {:>8.2}{: >4}\n{: <8}: {:>8.2}{: >4}\n{: <8}: {:>8.2}{: >4}\n",
                    "Temp",
                    m.temperature,
                    "\u{00B0}C",
                    "Pressure",
                    m.pressure,
                    "HPa",
                    "Humidity",
                    m.humidity,
                    "%"
                )
                .unwrap();

                if let Err(e) = Text::new(&txt_buf, Point::new(10, 10), txt_style).draw(&mut lcd) {
                    defmt::error!("Failed to draw: {:?}", Debug2Format(&e));
                }
            }
            Err(e) => {
                defmt::error!("Failed to get readings: {:?}", e);
            }
        }
        Timer::after_millis(500).await;
    }
}

#[embassy_executor::task]
async fn advertise_loop(
    mut peripheral: Peripheral<'static, BLECtrl<'static, 20>, DefaultPacketPool>,
    sensor: &'static Mutex<CriticalSectionRawMutex, BME280<SPIDevice<'static>, Delay>>,
) {
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "ESP32-BME280",
        appearance: &appearance::sensor::TEMPERATURE_SENSOR,
    }))
    .unwrap();

    loop {
        match advertise("ESP32-BME280", &mut peripheral, &server).await {
            Ok(conn) => {
                defmt::info!("Connection established");

                // Set up tasks for this connection
                let gatt_task = gatt_events_task(&server, &conn);
                let sensor_task = sensor_notify_task(&server, &conn, sensor);

                // Run until connection closes
                select(gatt_task, sensor_task).await;

                defmt::info!("Connection closed, restarting advertising");
            }
            Err(e) => {
                defmt::error!("Advertising error: {:?}", Debug2Format(&e));
            }
        }
    }
}

#[embassy_executor::task]
async fn ble_task(mut runner: Runner<'static, BLECtrl<'static, 20>, DefaultPacketPool>) {
    loop {
        if let Err(e) = runner.run().await {
            defmt::error!("Error running BLE: {:?}", Debug2Format(&e));
        }
    }
}

/// Stream Events until the connection closes.
///
/// This function will handle the GATT events and process them.
/// This is how we interact with read and write requests.
async fn gatt_events_task<P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
) -> Result<(), Error> {
    let rx_handle = &server.nus.rx;
    let reason = loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => break reason,
            GattConnectionEvent::Gatt { event } => {
                match &event {
                    GattEvent::Read(event) => {
                        if event.handle() == rx_handle.handle {
                            defmt::debug!("[gatt] Read event on handle: {:?}", rx_handle.handle);
                        }
                    }
                    GattEvent::Write(event) => {
                        if event.handle() == rx_handle.handle {
                            defmt::info!("[gatt] Received on RX: {:?}", event.data());
                        }
                    }
                    _ => {}
                };
                // This step is also performed at drop(), but writing it explicitly is necessary
                // in order to ensure reply is sent.
                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(e) => defmt::warn!("[gatt] error sending response: {:?}", Debug2Format(&e)),
                };
            }
            _ => {} // ignore other Gatt Connection Events
        }
    };
    defmt::info!("[gatt] disconnected: {:?}", reason);
    Ok(())
}

/// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
async fn advertise<'values, 'server, C: Controller>(
    name: &'values str,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut advertiser_data = [0; 128];
    let Uuid::Uuid128(id) = uuid!("6e400001-b5a3-f393-e0a9-e50e24dcca9e") else {
        unreachable!()
    };
    let len = AdStructure::encode_slice(
        &[
            AdStructure::CompleteLocalName(name.as_bytes()),
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids128(&[id]),
        ],
        &mut advertiser_data[..],
    )?;
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..len],
                scan_data: &[],
            },
        )
        .await?;
    defmt::info!("[adv] advertising");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    defmt::info!("[adv] connection established");
    Ok(conn)
}

/// Task to read sensor and send notifications
async fn sensor_notify_task(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    sensor: &'static Mutex<CriticalSectionRawMutex, BME280<SPIDevice<'static>, Delay>>,
) {
    let tx_char = &server.nus.tx;
    loop {
        Timer::after_millis(5000).await;

        // Read sensor
        let mut guard = sensor.lock().await;
        match guard.read_all(TempUnit::Celsius, PresUnit::HPa) {
            Ok(measurements) => {
                drop(guard);
                let mut buf: heapless::Vec<u8, NUS_BUF_SIZE> = heapless::Vec::new();
                let data: serde_json_core::heapless::String<NUS_BUF_SIZE> =
                    match serde_json_core::to_string(&measurements) {
                        Ok(v) => v,
                        Err(e) => {
                            defmt::error!("Failed to serialize: {:?}", e);
                            continue;
                        }
                    };

                defmt::debug!("Sending: {}", data.as_str());

                buf.extend_from_slice(data.as_bytes()).unwrap();

                // Send notification
                if let Err(e) = tx_char.notify(conn, &buf).await {
                    defmt::error!("Notify failed: {:?}", Debug2Format(&e));
                    break; // Connection likely dropped
                }
            }
            Err(e) => {
                defmt::error!("BME280 read failed: {}", e);
            }
        }
    }
}

use embassy_time::Timer;

use crate::bme280::{BME280, PresUnit, TempUnit};

#[embassy_executor::task]
pub async fn sensor_task(mut bme280: BME280<'static>) {
    if let Err(e) = bme280.init().await {
        defmt::error!("Failed to init sensor: {:?}", e);
        return;
    }

    loop {
        match bme280.read_all(TempUnit::Celsius, PresUnit::HPa).await {
            Ok(m) => {
                defmt::info!("Measurements: {}", m);
            }
            Err(e) => defmt::error!("Failed to read measurements: {:?}", e),
        }
        Timer::after_secs(1).await;
    }
}

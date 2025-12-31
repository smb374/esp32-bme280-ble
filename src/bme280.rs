#![allow(dead_code)]

use embedded_hal::{delay::DelayNs, spi::SpiDevice};
use serde_derive::{Deserialize, Serialize};

// Register addresses
const REG_CHIP_ID: u8 = 0xD0;
const REG_RESET: u8 = 0xE0;
const REG_CTRL_HUM: u8 = 0xF2;
const REG_STATUS: u8 = 0xF3;
const REG_CTRL_MEAS: u8 = 0xF4;
const REG_CONFIG: u8 = 0xF5;
const REG_PRESS_MSB: u8 = 0xF7;
const REG_TEMP_MSB: u8 = 0xFA;
const REG_HUM_MSB: u8 = 0xFD;

// Calibration data addresses
const REG_TEMP_DIG: u8 = 0x88; // 6 bytes
const REG_PRESS_DIG: u8 = 0x8E; // 18 bytes
const REG_HUM_DIG_1: u8 = 0xA1; // 1 byte
const REG_HUM_DIG_2: u8 = 0xE1; // 7 bytes

const RESET_VALUE: u8 = 0xB6;
const CHIP_ID_BME280: u8 = 0x60;
const CHIP_ID_BMP280: u8 = 0x58;

#[derive(Debug, Clone, Copy, defmt::Format)]
#[repr(u8)]
pub enum Oversampling {
    Off = 0,
    X1 = 1,
    X2 = 2,
    X4 = 3,
    X8 = 4,
    X16 = 5,
}

#[derive(Debug, Clone, Copy, defmt::Format)]
#[repr(u8)]
pub enum Mode {
    Sleep = 0,
    Forced = 1,
    Normal = 3,
}

#[derive(Debug, Clone, Copy, defmt::Format)]
#[repr(u8)]
pub enum StandbyTime {
    Us500 = 0,
    Us62500 = 1,
    Ms125 = 2,
    Ms250 = 3,
    Ms50 = 4,
    Ms1000 = 5,
    Ms10 = 6,
    Ms20 = 7,
}

#[derive(Debug, Clone, Copy, defmt::Format)]
#[repr(u8)]
pub enum Filter {
    Off = 0,
    F2 = 1,
    F4 = 2,
    F8 = 3,
    F16 = 4,
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum ChipModel {
    Unknown,
    BMP280,
    BME280,
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum TempUnit {
    Celsius,
    Fahrenheit,
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum PresUnit {
    Pa,
    HPa,
    InHg,
    Atm,
    Bar,
    MBar,
    Torr,
    Psi,
}

#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub temp_oversample: Oversampling,
    pub hum_oversample: Oversampling,
    pub pres_oversample: Oversampling,
    pub mode: Mode,
    pub standby_time: StandbyTime,
    pub filter: Filter,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            temp_oversample: Oversampling::X2,
            hum_oversample: Oversampling::X1,
            pres_oversample: Oversampling::X16,
            mode: Mode::Forced,
            standby_time: StandbyTime::Ms1000,
            filter: Filter::Off,
        }
    }
}

#[derive(Debug, Clone, Copy, defmt::Format, Serialize, Deserialize)]
pub struct Measurements {
    #[serde(rename = "t")]
    pub temperature: f32,
    #[serde(rename = "p")]
    pub pressure: f32,
    #[serde(rename = "h")]
    pub humidity: f32,
}

// Calibration data storage
struct CalibrationData {
    dig: [u8; 32],
}

impl CalibrationData {
    fn new() -> Self {
        Self { dig: [0u8; 32] }
    }

    // Temperature calibration
    fn dig_t1(&self) -> u16 {
        u16::from_le_bytes([self.dig[0], self.dig[1]])
    }

    fn dig_t2(&self) -> i16 {
        i16::from_le_bytes([self.dig[2], self.dig[3]])
    }

    fn dig_t3(&self) -> i16 {
        i16::from_le_bytes([self.dig[4], self.dig[5]])
    }

    // Pressure calibration
    fn dig_p1(&self) -> u16 {
        u16::from_le_bytes([self.dig[6], self.dig[7]])
    }

    fn dig_p2(&self) -> i16 {
        i16::from_le_bytes([self.dig[8], self.dig[9]])
    }

    fn dig_p3(&self) -> i16 {
        i16::from_le_bytes([self.dig[10], self.dig[11]])
    }

    fn dig_p4(&self) -> i16 {
        i16::from_le_bytes([self.dig[12], self.dig[13]])
    }

    fn dig_p5(&self) -> i16 {
        i16::from_le_bytes([self.dig[14], self.dig[15]])
    }

    fn dig_p6(&self) -> i16 {
        i16::from_le_bytes([self.dig[16], self.dig[17]])
    }

    fn dig_p7(&self) -> i16 {
        i16::from_le_bytes([self.dig[18], self.dig[19]])
    }

    fn dig_p8(&self) -> i16 {
        i16::from_le_bytes([self.dig[20], self.dig[21]])
    }

    fn dig_p9(&self) -> i16 {
        i16::from_le_bytes([self.dig[22], self.dig[23]])
    }

    // Humidity calibration
    fn dig_h1(&self) -> u8 {
        self.dig[24]
    }

    fn dig_h2(&self) -> i16 {
        i16::from_le_bytes([self.dig[25], self.dig[26]])
    }

    fn dig_h3(&self) -> u8 {
        self.dig[27]
    }

    fn dig_h4(&self) -> i16 {
        ((self.dig[28] as i8 as i16) << 4) | ((self.dig[29] as i16) & 0x0F)
    }

    fn dig_h5(&self) -> i16 {
        ((self.dig[30] as i8 as i16) << 4) | (((self.dig[29] as i16) >> 4) & 0x0F)
    }

    fn dig_h6(&self) -> i8 {
        self.dig[31] as i8
    }
}

pub struct BME280<SPI: SpiDevice, D: DelayNs> {
    spi: SPI,
    delay: D,
    config: Config,
    calib: CalibrationData,
    chip_model: ChipModel,
    initialized: bool,
}

impl<SPI: SpiDevice, D: DelayNs> BME280<SPI, D> {
    pub fn new(spi: SPI, delay: D, config: Config) -> Self {
        Self {
            spi,
            delay,
            config,
            calib: CalibrationData::new(),
            chip_model: ChipModel::Unknown,
            initialized: false,
        }
    }

    pub fn init(&mut self) -> Result<(), &'static str> {
        // Read and verify chip ID
        let chip_id = self
            .read_register(REG_CHIP_ID)
            .map_err(|_| "Failed to read chip ID")?;

        defmt::info!("BME280 Chip ID: {:#02X}", chip_id);

        self.chip_model = match chip_id {
            CHIP_ID_BME280 => ChipModel::BME280,
            CHIP_ID_BMP280 => ChipModel::BMP280,
            _ => return Err("Unknown chip ID"),
        };

        // Read calibration data
        self.read_calibration()?;

        // Initialize filter if needed
        if !matches!(self.config.filter, Filter::Off) {
            self.initialize_filter()?;
        }

        // Write settings
        self.write_settings()?;

        self.initialized = true;
        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), &'static str> {
        self.write_register(REG_RESET, RESET_VALUE)
            .map_err(|_| "Failed to reset")?;
        self.delay.delay_ms(2);
        self.init()
    }

    fn initialize_filter(&mut self) -> Result<(), &'static str> {
        // Force unfiltered measurement to populate filter buffer
        let original_filter = self.config.filter;
        self.config.filter = Filter::Off;

        self.write_settings()?;

        // Read dummy measurement
        let _ = self.read_all(TempUnit::Celsius, PresUnit::HPa);

        self.config.filter = original_filter;
        Ok(())
    }

    fn read_calibration(&mut self) -> Result<(), &'static str> {
        let mut calib_data = [0u8; 32];

        // Temperature and pressure calibration (0x88-0x9F = 24 bytes)
        self.read_registers(REG_TEMP_DIG, &mut calib_data[0..6])
            .map_err(|_| "Failed to read temp calib")?;

        self.read_registers(REG_PRESS_DIG, &mut calib_data[6..24])
            .map_err(|_| "Failed to read press calib")?;

        // Humidity calibration part 1 (0xA1 = 1 byte)
        self.read_registers(REG_HUM_DIG_1, &mut calib_data[24..25])
            .map_err(|_| "Failed to read hum calib 1")?;

        // Humidity calibration part 2 (0xE1-0xE7 = 7 bytes)
        self.read_registers(REG_HUM_DIG_2, &mut calib_data[25..32])
            .map_err(|_| "Failed to read hum calib 2")?;

        // Copy to self.calib.dig
        self.calib.dig.copy_from_slice(&calib_data);

        defmt::debug!("Calibration data loaded");
        Ok(())
    }

    fn write_settings(&mut self) -> Result<(), &'static str> {
        let ctrl_hum = self.config.hum_oversample as u8;

        let ctrl_meas = ((self.config.temp_oversample as u8) << 5)
            | ((self.config.pres_oversample as u8) << 2)
            | (self.config.mode as u8);

        let config = ((self.config.standby_time as u8) << 5) | ((self.config.filter as u8) << 2);

        self.write_register(REG_CTRL_HUM, ctrl_hum)
            .map_err(|_| "Failed to write CTRL_HUM")?;

        self.write_register(REG_CTRL_MEAS, ctrl_meas)
            .map_err(|_| "Failed to write CTRL_MEAS")?;

        self.write_register(REG_CONFIG, config)
            .map_err(|_| "Failed to write CONFIG")?;

        Ok(())
    }

    fn read_data(&mut self) -> Result<[u8; 8], &'static str> {
        // For forced mode, trigger measurement
        if matches!(self.config.mode, Mode::Forced) {
            self.write_settings()?;
        }

        let mut data = [0u8; 8];
        self.read_registers(REG_PRESS_MSB, &mut data)
            .map_err(|_| "Failed to read sensor data")?;

        Ok(data)
    }

    pub fn read_temperature(&mut self, unit: TempUnit) -> Result<f32, &'static str> {
        let data = self.read_data()?;
        let raw_temp = ((data[3] as u32) << 12) | ((data[4] as u32) << 4) | ((data[5] as u32) >> 4);

        let mut t_fine = 0i32;
        Ok(self.calculate_temperature(raw_temp as i32, &mut t_fine, unit))
    }

    pub fn read_pressure(&mut self, unit: PresUnit) -> Result<f32, &'static str> {
        let data = self.read_data()?;
        let raw_temp = ((data[3] as u32) << 12) | ((data[4] as u32) << 4) | ((data[5] as u32) >> 4);
        let raw_pres = ((data[0] as u32) << 12) | ((data[1] as u32) << 4) | ((data[2] as u32) >> 4);

        let mut t_fine = 0i32;
        self.calculate_temperature(raw_temp as i32, &mut t_fine, TempUnit::Celsius);
        Ok(self.calculate_pressure(raw_pres as i32, t_fine, unit))
    }

    pub fn read_humidity(&mut self) -> Result<f32, &'static str> {
        let data = self.read_data()?;
        let raw_temp = ((data[3] as u32) << 12) | ((data[4] as u32) << 4) | ((data[5] as u32) >> 4);
        let raw_hum = ((data[6] as u32) << 8) | (data[7] as u32);

        let mut t_fine = 0i32;
        self.calculate_temperature(raw_temp as i32, &mut t_fine, TempUnit::Celsius);
        Ok(self.calculate_humidity(raw_hum as i32, t_fine))
    }

    pub fn read_all(
        &mut self,
        temp_unit: TempUnit,
        pres_unit: PresUnit,
    ) -> Result<Measurements, &'static str> {
        let data = self.read_data()?;

        let raw_pres = ((data[0] as u32) << 12) | ((data[1] as u32) << 4) | ((data[2] as u32) >> 4);
        let raw_temp = ((data[3] as u32) << 12) | ((data[4] as u32) << 4) | ((data[5] as u32) >> 4);
        let raw_hum = ((data[6] as u32) << 8) | (data[7] as u32);

        let mut t_fine = 0i32;
        let temperature = self.calculate_temperature(raw_temp as i32, &mut t_fine, temp_unit);
        let pressure = self.calculate_pressure(raw_pres as i32, t_fine, pres_unit);
        let humidity = self.calculate_humidity(raw_hum as i32, t_fine);

        Ok(Measurements {
            temperature,
            pressure,
            humidity,
        })
    }

    pub fn chip_model(&self) -> ChipModel {
        self.chip_model
    }

    // Compensation algorithms from Bosch datasheet
    fn calculate_temperature(&self, raw: i32, t_fine: &mut i32, unit: TempUnit) -> f32 {
        let dig_t1 = self.calib.dig_t1() as i32;
        let dig_t2 = self.calib.dig_t2() as i32;
        let dig_t3 = self.calib.dig_t3() as i32;

        let var1 = (((raw >> 3) - (dig_t1 << 1)) * dig_t2) >> 11;
        let var2 = (((((raw >> 4) - dig_t1) * ((raw >> 4) - dig_t1)) >> 12) * dig_t3) >> 14;

        *t_fine = var1 + var2;
        let temp_celsius = ((*t_fine * 5 + 128) >> 8) as f32 / 100.0;

        match unit {
            TempUnit::Celsius => temp_celsius,
            TempUnit::Fahrenheit => temp_celsius * 9.0 / 5.0 + 32.0,
        }
    }

    fn calculate_humidity(&self, raw: i32, t_fine: i32) -> f32 {
        let dig_h1 = self.calib.dig_h1() as i32;
        let dig_h2 = self.calib.dig_h2() as i32;
        let dig_h3 = self.calib.dig_h3() as i32;
        let dig_h4 = self.calib.dig_h4() as i32;
        let dig_h5 = self.calib.dig_h5() as i32;
        let dig_h6 = self.calib.dig_h6() as i32;

        let mut var1 = t_fine - 76800;
        var1 = ((((raw << 14) - (dig_h4 << 20) - (dig_h5 * var1)) + 16384) >> 15)
            * (((((((var1 * dig_h6) >> 10) * (((var1 * dig_h3) >> 11) + 32768)) >> 10) + 2097152)
                * dig_h2
                + 8192)
                >> 14);
        var1 = var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * dig_h1) >> 4);
        var1 = if var1 < 0 { 0 } else { var1 };
        var1 = if var1 > 419430400 { 419430400 } else { var1 };

        ((var1 >> 12) as u32 as f32) / 1024.0
    }

    fn calculate_pressure(&self, raw: i32, t_fine: i32, unit: PresUnit) -> f32 {
        let dig_p1 = self.calib.dig_p1() as i64;
        let dig_p2 = self.calib.dig_p2() as i64;
        let dig_p3 = self.calib.dig_p3() as i64;
        let dig_p4 = self.calib.dig_p4() as i64;
        let dig_p5 = self.calib.dig_p5() as i64;
        let dig_p6 = self.calib.dig_p6() as i64;
        let dig_p7 = self.calib.dig_p7() as i64;
        let dig_p8 = self.calib.dig_p8() as i64;
        let dig_p9 = self.calib.dig_p9() as i64;

        let mut var1 = (t_fine as i64) - 128000;
        let mut var2 = var1 * var1 * dig_p6;
        var2 = var2 + ((var1 * dig_p5) << 17);
        var2 = var2 + (dig_p4 << 35);
        var1 = ((var1 * var1 * dig_p3) >> 8) + ((var1 * dig_p2) << 12);
        var1 = ((1i64 << 47) + var1) * dig_p1 >> 33;

        if var1 == 0 {
            return 0.0; // Avoid division by zero
        }

        let mut pressure = 1048576 - raw as i64;
        pressure = (((pressure << 31) - var2) * 3125) / var1;
        var1 = (dig_p9 * (pressure >> 13) * (pressure >> 13)) >> 25;
        var2 = (dig_p8 * pressure) >> 19;
        pressure = ((pressure + var1 + var2) >> 8) + (dig_p7 << 4);

        let pressure_pa = (pressure as u32 as f32) / 256.0;

        match unit {
            PresUnit::Pa => pressure_pa,
            PresUnit::HPa => pressure_pa / 100.0,
            PresUnit::InHg => pressure_pa / 3386.3752577878,
            PresUnit::Atm => pressure_pa / 101324.99766353,
            PresUnit::Bar => pressure_pa / 100000.0,
            PresUnit::MBar => pressure_pa / 100.0,
            PresUnit::Torr => pressure_pa / 133.32236534674,
            PresUnit::Psi => pressure_pa / 6894.744825494,
        }
    }

    // Low-level SPI operations
    fn read_register(&mut self, reg: u8) -> Result<u8, ()> {
        let mut buf = [reg | 0x80, 0x00];
        self.spi.transfer_in_place(&mut buf).map_err(|_| ())?;
        Ok(buf[1])
    }

    fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), ()> {
        let mut tx_buf = [0u8; 64];
        tx_buf[0] = reg | 0x80;

        let total_len = buf.len() + 1;
        self.spi
            .transfer_in_place(&mut tx_buf[..total_len])
            .map_err(|_| ())?;
        buf.copy_from_slice(&tx_buf[1..total_len]);
        Ok(())
    }

    fn write_register(&mut self, reg: u8, value: u8) -> Result<(), ()> {
        let buf = [reg & 0x7F, value];
        self.spi.write(&buf).map_err(|_| ())?;
        Ok(())
    }
}

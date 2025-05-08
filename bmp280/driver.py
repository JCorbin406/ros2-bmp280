# bmp280/driver.py

import time
from smbus2 import SMBus

class BMP280:
    """Driver for the BMP280 temperature and pressure sensor over I2C."""

    def __init__(self, bus: int = 1, address: int = 0x76):
        """
        Initialize the BMP280 sensor.

        Args:
            bus (int): I2C bus number.
            address (int): I2C address of the BMP280 (0x76 or 0x77).
        """
        self.bus = SMBus(bus)
        self.address = address

        # Read calibration data from the sensor
        self.dig_T = []
        self.dig_P = []
        self._read_calibration_data()

        # Configure the sensor (normal mode, oversampling x1)
        self.bus.write_byte_data(self.address, 0xF4, 0x27)
        self.bus.write_byte_data(self.address, 0xF5, 0xA0)
        time.sleep(0.1)

    def _read_calibration_data(self):
        """Read temperature and pressure calibration data from the sensor."""
        calib = self.bus.read_i2c_block_data(self.address, 0x88, 24)

        self.dig_T = [
            self._u16(calib[1], calib[0]),
            self._s16(calib[3], calib[2]),
            self._s16(calib[5], calib[4])
        ]

        self.dig_P = [
            self._u16(calib[7], calib[6]),
            self._s16(calib[9], calib[8]),
            self._s16(calib[11], calib[10]),
            self._s16(calib[13], calib[12]),
            self._s16(calib[15], calib[14]),
            self._s16(calib[17], calib[16]),
            self._s16(calib[19], calib[18]),
            self._s16(calib[21], calib[20]),
            self._s16(calib[23], calib[22])
        ]

    def read_raw_data(self):
        """Read raw temperature and pressure data from sensor."""
        data = self.bus.read_i2c_block_data(self.address, 0xF7, 6)
        adc_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        adc_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        return adc_t, adc_p

    def read_temperature_pressure(self):
        """Compute and return calibrated temperature (°C) and pressure (Pa)."""
        adc_t, adc_p = self.read_raw_data()

        # Temperature compensation
        var1 = (((adc_t >> 3) - (self.dig_T[0] << 1)) * self.dig_T[1]) >> 11
        var2 = (((((adc_t >> 4) - self.dig_T[0]) * ((adc_t >> 4) - self.dig_T[0])) >> 12) * self.dig_T[2]) >> 14
        t_fine = var1 + var2
        temperature = (t_fine * 5 + 128) >> 8
        temperature /= 100.0

        # Pressure compensation
        var1 = t_fine - 128000
        var2 = var1 * var1 * self.dig_P[5]
        var2 = var2 + ((var1 * self.dig_P[4]) << 17)
        var2 = var2 + (self.dig_P[3] << 35)
        var1 = ((var1 * var1 * self.dig_P[2]) >> 8) + ((var1 * self.dig_P[1]) << 12)
        var1 = (((1 << 47) + var1) * self.dig_P[0]) >> 33

        if var1 == 0:
            pressure = 0  # Avoid division by zero
        else:
            p = 1048576 - adc_p
            p = (((p << 31) - var2) * 3125) // var1
            var1 = (self.dig_P[8] * (p >> 13) * (p >> 13)) >> 25
            var2 = (self.dig_P[7] * p) >> 19
            pressure = ((p + var1 + var2) >> 8) + (self.dig_P[6] << 4)
            pressure /= 256.0

        return temperature, pressure

    @staticmethod
    def _u16(msb, lsb):
        return (msb << 8) | lsb

    @staticmethod
    def _s16(msb, lsb):
        result = (msb << 8) | lsb
        return result - 65536 if result > 32767 else result

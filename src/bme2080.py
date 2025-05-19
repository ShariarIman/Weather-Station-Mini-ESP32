from machine import I2C
import time

class BME280:
    def __init__(self, i2c, address=0x76, retries=3):
        self.i2c = i2c
        self.address = address
        self.retries = retries
        self.calibration_t = []
        self.calibration_p = []
        self.calibration_h = []
        self.t_fine = 0
        self._reset_and_setup()
        self.populate_calibration_data()

    def _reset_and_setup(self):
        # Soft reset
        try:
            self.i2c.writeto_mem(self.address, 0xE0, b'\xB6')
            time.sleep_ms(50)  # Wait for reset
        except OSError as e:
            print("Reset failed:", e)
        
        # Verify chip ID
        chip_id = self.i2c.readfrom_mem(self.address, 0xD0, 1)[0]
        print("Chip ID:", hex(chip_id), "(BME280 = 0x60)")
        if chip_id != 0x60:
            raise ValueError("Not a BME280 sensor")

        # Set to sleep mode
        self.i2c.writeto_mem(self.address, 0xF4, b'\x00')
        # Configure humidity oversampling (x1)
        self.i2c.writeto_mem(self.address, 0xF2, b'\x01')
        # Configure temp/pressure oversampling (x1), normal mode
        self.i2c.writeto_mem(self.address, 0xF4, b'\x27')
        # Configure standby time 0.5ms, filter off
        self.i2c.writeto_mem(self.address, 0xF5, b'\x00')
        
        # Verify humidity oversampling
        hum_ctrl = self.i2c.readfrom_mem(self.address, 0xF2, 1)[0]
        print("Humidity control (0xF2):", hum_ctrl)

    def populate_calibration_data(self):
        try:
            # Read all calibration data in one transaction
            raw_data = bytearray(32)
            raw_data[:25] = self.i2c.readfrom_mem(self.address, 0x88, 25)  # 0x88 to 0xA1
            raw_data[25:32] = self.i2c.readfrom_mem(self.address, 0xE1, 7)  # 0xE1 to 0xE7

            self.calibration_t.append((raw_data[1] << 8) | raw_data[0])
            self.calibration_t.append((raw_data[3] << 8) | raw_data[2])
            self.calibration_t.append((raw_data[5] << 8) | raw_data[4])
            self.calibration_p.append((raw_data[7] << 8) | raw_data[6])
            self.calibration_p.append((raw_data[9] << 8) | raw_data[8])
            self.calibration_p.append((raw_data[11] << 8) | raw_data[10])
            self.calibration_p.append((raw_data[13] << 8) | raw_data[12])
            self.calibration_p.append((raw_data[15] << 8) | raw_data[14])
            self.calibration_p.append((raw_data[17] << 8) | raw_data[16])
            self.calibration_p.append((raw_data[19] << 8) | raw_data[18])
            self.calibration_p.append((raw_data[21] << 8) | raw_data[20])
            self.calibration_p.append((raw_data[23] << 8) | raw_data[22])
            self.calibration_h.append(raw_data[24])
            self.calibration_h.append((raw_data[26] << 8) | raw_data[25])
            self.calibration_h.append(raw_data[27])
            self.calibration_h.append((raw_data[28] << 4) | (0x0F & raw_data[29]))
            self.calibration_h.append((raw_data[30] << 4) | ((raw_data[29] >> 4) & 0x0F))
            self.calibration_h.append(raw_data[31])

            for i in range(1, 3):
                if self.calibration_t[i] & 0x8000:
                    self.calibration_t[i] = (-self.calibration_t[i] ^ 0xFFFF) + 1
            for i in range(1, 9):
                if self.calibration_p[i] & 0x8000:
                    self.calibration_p[i] = (-self.calibration_p[i] ^ 0xFFFF) + 1
            for i in range(6):
                if self.calibration_h[i] & 0x8000:
                    self.calibration_h[i] = (-self.calibration_h[i] ^ 0xFFFF) + 1

            print("Humidity calibration:", self.calibration_h)
        except OSError as e:
            print("Calibration read failed:", e)
            raise

    def compensate_temperature(self, adc_t):
        v1 = (adc_t / 16384.0 - self.calibration_t[0] / 1024.0) * self.calibration_t[1]
        v2 = (adc_t / 131072.0 - self.calibration_t[0] / 8192.0) ** 2 * self.calibration_t[2]
        self.t_fine = v1 + v2
        return self.t_fine / 5120.0

    def compensate_pressure(self, adc_p):
        v1 = (self.t_fine / 2.0) - 64000.0
        v2 = ((v1 / 4.0) ** 2 / 2048) * self.calibration_p[5]
        v2 += (v1 * self.calibration_p[4]) * 2.0
        v2 = (v2 / 4.0) + (self.calibration_p[3] * 65536.0)
        v1 = ((self.calibration_p[2] * ((v1 / 4.0) ** 2 / 8192) / 8) + (self.calibration_p[1] * v1) / 2.0) / 262144
        v1 = ((32768 + v1) * self.calibration_p[0]) / 32768

        if v1 == 0:
            return 0

        pressure = ((1048576 - adc_p) - (v2 / 4096)) * 3125
        pressure = (pressure * 2.0 / v1) if pressure < 0x80000000 else (pressure / v1) * 2
        v1 = (self.calibration_p[8] * ((pressure / 8.0) ** 2 / 8192.0)) / 4096
        v2 = (pressure / 4.0 * self.calibration_p[7]) / 8192.0
        return (pressure + (v1 + v2 + self.calibration_p[6]) / 16.0) / 100  # hPa

    def compensate_humidity(self, adc_h):
        var_h = self.t_fine - 76800.0
        if var_h == 0 or adc_h == 0:
            return 0.0

        var_h = (adc_h - (self.calibration_h[3] * 64.0 + self.calibration_h[4] / 16384.0 * var_h)) * (
            self.calibration_h[1] / 65536.0 * (1.0 + self.calibration_h[5] / 67108864.0 * var_h * (
                1.0 + self.calibration_h[2] / 67108864.0 * var_h)))
        var_h *= (1.0 - self.calibration_h[0] * var_h / 524288.0)

        return min(max(var_h, 0.0), 100.0)

    def read_raw(self):
        for _ in range(self.retries):
            try:
                data = self.i2c.readfrom_mem(self.address, 0xF7, 8)
                pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
                temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
                hum_raw = (data[6] << 8) | data[7]
                if temp_raw > 0 and pres_raw > 0:  # Basic validation
                    return temp_raw, pres_raw, hum_raw
            except OSError as e:
                print("I2C read error:", e)
            time.sleep_ms(10)
        raise OSError("Failed to read BME280 after retries")

    def values(self):
        try:
            temp_raw, pres_raw, hum_raw = self.read_raw()
            temp = self.compensate_temperature(temp_raw)
            pres = self.compensate_pressure(pres_raw)
            hum = self.compensate_humidity(hum_raw)
            return temp, pres, hum
        except OSError as e:
            print("Sensor read failed:", e)
            return None, None, None

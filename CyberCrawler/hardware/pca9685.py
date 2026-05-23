from machine import I2C

class PCA9685:
    def __init__(self, i2c, address=0x40):
        self.i2c = i2c
        self.address = address
        self.reset()

    def write(self, reg, value):
        self.i2c.writeto_mem(self.address, reg, bytes([value]))

    def read(self, reg):
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]

    def reset(self):
        self.write(0x00, 0x00)  # MODE1

    def set_pwm_freq(self, freq):
        prescale = int(25000000.0 / 4096 / freq - 1)
        old_mode = self.read(0x00)
        self.write(0x00, (old_mode & 0x7F) | 0x10)
        self.write(0xFE, prescale)
        self.write(0x00, old_mode)
        self.write(0x00, old_mode | 0xA1)

    def set_pwm(self, ch, on, off):
        reg = 0x06 + 4 * ch
        self.i2c.writeto_mem(self.address, reg,
            bytes([on & 0xFF, on >> 8, off & 0xFF, off >> 8]))
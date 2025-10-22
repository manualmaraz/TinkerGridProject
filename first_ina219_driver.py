# ina219.py for Raspberry Pi Pico
#file before awg implementation
# This file contains the INA219 class definition.
# Upload this file to your Pico's filesystem.

# INA219 I2C Address (default)
INA219_ADDRESS = 0x40 # Common default address for INA219

# INA219 Registers
REG_CONFIG = 0x00
REG_SHUNTVOLTAGE = 0x01
REG_BUSVOLTAGE = 0x02
REG_POWER = 0x03
REG_CURRENT = 0x04
REG_CALIBRATION = 0x05

# Calibration values (for 32V, 2A range with 0.1 Ohm shunt)
# This is a common default for many INA219 modules.
# If your module has a different shunt resistor or you need to measure
# different ranges (e.g., higher current, lower voltage for more resolution),
# you MUST re-calculate CALIBRATION_VALUE and CURRENT_LSB.
# Refer to INA219 datasheet or Adafruit INA219 guide for calculations.
CALIBRATION_VALUE = 4096  # Example for 32V, 2A, 0.1 Ohm shunt
CURRENT_LSB = 0.0001      # 100uA per bit for the above calibration

class INA219:
    def __init__(self, i2c_bus, addr=INA219_ADDRESS):
        self.i2c = i2c_bus
        self.address = addr
        # Default configuration: 32V, 2A, 12-bit ADC conversions, averaging
        # This sets the Bus Voltage Range (BVR), Shunt Voltage Range (SVR),
        # ADC resolution for bus and shunt, and operation mode (continuous).
        # 0x399F -> (Bus Voltage Range=32V, Shunt Voltage Range=+-320mV,
        #            Bus ADC=12bit 532us, Shunt ADC=12bit 532us, Mode=Shunt and Bus, Continuous)
        self._write_word(REG_CONFIG, 0x399F)
        self._write_word(REG_CALIBRATION, CALIBRATION_VALUE)

    def _read_word(self, register):
        # Reads a 16-bit word from a register (INA219 sends MSB first)
        data = self.i2c.readfrom_mem(self.address, register, 2)
        val = (data[0] << 8) | data[1]
        # Convert to signed integer if the value is negative
        if val > 0x7FFF: # If sign bit is set (for 16-bit value)
            val -= 0x10000
        return val

    def _write_word(self, register, value):
        # Writes a 16-bit word to a register (MSB first)
        msb = (value >> 8) & 0xFF
        lsb = value & 0xFF
        self.i2c.writeto_mem(self.address, register, bytes([msb, lsb]))

    def get_shunt_voltage_mV(self):
        # Shunt voltage is in increments of 10uV (0.01mV)
        return self._read_word(REG_SHUNTVOLTAGE) * 0.01

    def get_bus_voltage_V(self):
        # Bus voltage is in increments of 4mV (0.004V)
        # Raw value needs to be shifted right by 3 bits as per datasheet (LSB is 4mV)
        raw_bus_voltage = self._read_word(REG_BUSVOLTAGE) >> 3
        return raw_bus_voltage * 0.004

    def get_current_mA(self):
        # Current is calculated from the current register value * CURRENT_LSB
        # The CURRENT_LSB is determined by the calibration settings.
        return self._read_word(REG_CURRENT) * CURRENT_LSB * 1000 # Convert to mA

    def get_power_mW(self):
        # Power is calculated from the power register value * Power LSB
        # Power LSB = 20 * Current LSB (as per INA219 datasheet when using REG_POWER directly)
        return self._read_word(REG_POWER) * (CURRENT_LSB * 20) * 1000 # Convert to mW

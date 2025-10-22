# main.py
# MicroPython code for Raspberry Pi Pico to control the AD9833 DDS module.

from machine import Pin, SPI
from time import sleep

# --- AD9833 Pin Definitions (Modified to use SPI1) ---
# Switching to GPIO 20 for FSYNC to test a third, isolated pin.
# This is a general-purpose digital test to ensure the Pico can drive the pin low.
# SPI BUS = 1 (SPI1)
FSYNC_PIN = 20  # Pin 26 - Chip Select (New Test Pin)
SCK_PIN = 10 # Pin 14 - SPI1 SCK
SDATA_PIN = 11 # Pin 15 - SPI1 MOSI

# --- AD9833 Register Definitions and Constants ---
# Master Clock Frequency (usually 25 MHz for standard modules)
MCLK = 25000000.0
FREQ_BITS = 2**28  # 2^28

# Control Register Commands (16-bit words)
# D8: RESET bit (1=reset, 0=normal operation)
CMD_RESET = 0x0100
# D13: B28 (1=28-bit frequency writes, 0=14-bit)
# D12: Selects FREG0/1/PHASE0/1 registers. Set to 1 for writing FREG0/1
CMD_CONTROL_START = 0x2000 # Select 28-bit writes, use FREG0/1 (D13=1)
# D15-D14: FREG select bits (FREG0=01, FREG1=10)
REG_FREQ0_ADDR = 0x4000
REG_FREQ1_ADDR = 0x8000

# Waveform Select (D5, D3, D1)
MODE_SINE = 0x0000
MODE_TRIANGLE = 0x0002
MODE_SQUARE = 0x0028 # Square wave mode (D5=1, D4=0, D3=1)

# --- AD9833 Control Class ---

class AD9833:
    """Class to manage the AD9833 DDS module."""

    def __init__(self, sck_pin, mosi_pin, fsync_pin, mclk=MCLK):
        self.mclk = mclk

        # Initialize SPI bus 1 (SPI1 is used here)
        self.spi = SPI(1, baudrate=10000000, polarity=0, phase=0,
                       sck=Pin(sck_pin), mosi=Pin(mosi_pin))

        # FSYNC (Chip Select) pin must be driven low to send data
        # We initialize it here without the pull-up for cleaner driving
        self.fsync = Pin(fsync_pin, Pin.OUT)
        self.fsync.value(1) # Keep FSYNC high (inactive) initially

        # Buffers for sending 16-bit commands
        self.tx_buf = bytearray(2)

        self._initialize_chip()

    def _write_command(self, command):
        """Sends a 16-bit command word via SPI."""
        # Convert 16-bit integer to two 8-bit bytes (MSB first)
        self.tx_buf[0] = (command >> 8) & 0xFF
        self.tx_buf[1] = command & 0xFF

        self.fsync.value(0) # Drive FSYNC low to start transmission
        self.spi.write(self.tx_buf)
        self.fsync.value(1) # Drive FSYNC high to end transmission
        sleep(0.00001) # Small delay to ensure command completes

    def _initialize_chip(self):
        """Initializes the AD9833 into 28-bit mode with a default frequency."""
        
        # 1. Hardware Reset: Set D8 (RESET) to 1. Puts chip into reset mode.
        self._write_command(CMD_RESET)
        
        # 2. Control Word 1: Set D13 (B28) to 1 to enable 28-bit frequency writes.
        # This tells the chip that the next two writes will be the LSB/MSB of a FREG.
        self._write_command(CMD_CONTROL_START)
        
        # 3. Load Default Frequency (FREG0)
        self.set_frequency(1000.0, reg=0)
        
        # 4. Control Word 2: Clear D8 (RESET) and set output mode.
        # This is the final step that takes the chip out of reset and starts generation.
        # 0x0000 means: RESET=0, Sleep=0, FREG0 active, Sine output.
        self._write_command(MODE_SINE | REG_FREQ0_ADDR)


    def set_frequency(self, frequency, reg=0):
        """
        Calculates and sets the output frequency for a selected register (0 or 1).
        """
        if frequency < 0 or frequency > (self.mclk / 2):
            print("Error: Frequency out of range.")
            return

        # Frequency Tuning Word Calculation: F_word = (F_out * 2^28) / F_mclk
        freq_word = int((frequency * FREQ_BITS) / self.mclk)

        # Split 28-bit word into two 14-bit words (LSB and MSB)
        LSB = freq_word & 0x3FFF  # Lower 14 bits
        MSB = (freq_word >> 14) & 0x3FFF # Upper 14 bits

        # Add register address and control bits (D15/D14)
        if reg == 0:
            LSB |= REG_FREQ0_ADDR
            MSB |= REG_FREQ0_ADDR
        elif reg == 1:
            LSB |= REG_FREQ1_ADDR
            MSB |= REG_FREQ1_ADDR
        else:
            print("Error: Invalid frequency register.")
            return

        # Write LSB word first, then MSB word
        self._write_command(LSB)
        self._write_command(MSB)

    def set_waveform(self, mode):
        """Sets the output waveform type (SINE, TRIANGLE, or SQUARE)."""
        # Note: We clear the D13 B28 bit here by only including the mode and FREG0 address
        self._write_command(mode | REG_FREQ0_ADDR) # Use FREG0

    def test_sweep(self):
        """
        Runs a test sequence: Sine, Triangle, and Square wave sweep.
        """
        print("--- Starting AD9833 Test Sweep ---")
        
        # Wait for 1kHz sine wave set in initialization
        print("Initial state: 1kHz Sine Wave (waiting 2s)")
        sleep(2)

        # 1. Sine Wave Sweep
        print("Mode: Sine Wave (100Hz to 10kHz)")
        self.set_waveform(MODE_SINE)
        for freq in range(100, 10001, 1000):
            self.set_frequency(float(freq))
            print(f"Setting Frequency: {freq} Hz")
            sleep(0.5)

        # 2. Triangle Wave Test
        print("Mode: Triangle Wave (5kHz)")
        self.set_waveform(MODE_TRIANGLE)
        self.set_frequency(5000.0)
        sleep(2)

        # 3. Square Wave Test
        print("Mode: Square Wave (50Hz)")
        self.set_waveform(MODE_SQUARE)
        self.set_frequency(50.0)
        sleep(2)

        # 4. Final Sine Wave
        print("Mode: Sine Wave (1.5kHz) - End of Sweep")
        self.set_waveform(MODE_SINE)
        self.set_frequency(1500.0)


# --- Main Execution ---
if __name__ == '__main__':
    # --- Critical Pin Drive Test (Visual LED Check) ---
    # This test verifies the Pico can drive the FSYNC pin (GPIO 20 / Pin 26) low.
    test_pin = Pin(FSYNC_PIN, Pin.OUT)
    print(f"--- Testing FSYNC Pin {FSYNC_PIN} (Pin 26) Drive ---")
    print(f"**LED TEST**: Connect LED (anode) to 3.3V, LED (cathode/resistor) to Pin {FSYNC_PIN}. The LED must blink!")
    
    # Manually toggle the pin 5 times (0.5s on, 0.5s off)
    for i in range(5):
        test_pin.value(0) # Drive Low (0V) - LED should turn ON
        sleep(0.5)
        test_pin.value(1) # Drive High (3.3V) - LED should turn OFF
        sleep(0.5)
    
    print("--- FSYNC Pin Test Complete. Starting AD9833 init. ---")
    
    # If the LED did NOT blink, the Pico pin itself is likely damaged or faulty.
    # Please verify the LED circuit works by temporarily connecting the resistor directly to GND.
    
    dds = None
    try:
        # Initialize the AD9833 driver
        dds = AD9833(SCK_PIN, SDATA_PIN, FSYNC_PIN)
        print(f"AD9833 initialized on SPI1 (SCK={SCK_PIN}, MOSI={SDATA_PIN}, FSYNC={FSYNC_PIN})")

        # Run the test sequence
        dds.test_sweep()

        # Keep the final signal active for continuous monitoring
        print("Test sequence finished. AD9833 is holding 1.5kHz Sine Wave.")
        i = 0
        while True:
            sleep(10)
            print(f"Status check {i+1}: 1.5kHz Sine Wave is active.")
            i += 1

    except Exception as e:
        print(f"An error occurred: {e}")
        print("CRITICAL: Check your wiring and power. The low voltage suggests a major power/ground issue.")
        print("Make sure the AD9833 module is receiving 3.3V and its GND is connected.")
        if dds:
            # Attempt to reset the chip if it was initialized
            dds._write_command(CMD_RESET)
            print("AD9833 reset attempted.")

    except KeyboardInterrupt:
        print("Program stopped by user.")
        if dds:
            dds._write_command(CMD_RESET)
            print("AD9833 reset on exit.")

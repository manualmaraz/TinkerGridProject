import machine
import utime
import math

# Pin definitions for Raspberry Pi Pico
FSYNC_PIN = 17      # GPIO17 - AD9833 Chip Select
SW1_PIN = 20        # GPIO20 - DIP Switch 1
SW2_PIN = 21        # GPIO21 - DIP Switch 2
FREQ_POT_PIN = 26   # GPIO26/ADC0 - Frequency potentiometer

# SPI pins (hardware SPI0)
SCK_PIN = 18        # GPIO18 - SPI Clock
MOSI_PIN = 19       # GPIO19 - SPI Data

# Frequency ranges
MIN_FREQ = 1.0
MAX_FREQ = 1000000.0  # 1MHz maximum

class AD9833_Working:
    def __init__(self):
        # Initialize pins
        self.fsync = machine.Pin(FSYNC_PIN, machine.Pin.OUT)
        self.sw1 = machine.Pin(SW1_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
        self.sw2 = machine.Pin(SW2_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
        self.freq_pot = machine.ADC(FREQ_POT_PIN)
        
        # Initialize SPI
        self.spi = machine.SPI(0, 
                              baudrate=1000000,
                              polarity=1, 
                              phase=1,
                              sck=machine.Pin(SCK_PIN),
                              mosi=machine.Pin(MOSI_PIN))
        
        print("=== AD9833 Working AWG - MicroPython ===")
        print("Potentiometer: Frequency Control (1Hz to 1MHz)")
        print("DIP Switches: Waveform Selection")
        print("")
        print("Waveform Selection Table:")
        print("SW1  SW2  | Waveform")
        print("OFF  OFF  | SINE")
        print("OFF  ON   | TRIANGLE")
        print("ON   OFF  | SQUARE")
        print("ON   ON   | SQUARE/2")
        print("========================================")
        
        # Initialize AD9833
        self.init_ad9833()
        
        # Track previous states
        self.prev_pot_value = None
        self.prev_sw1 = None
        self.prev_sw2 = None
        
        print("Ready! Use potentiometer and DIP switches")
    
    def init_ad9833(self):
        """Initialize AD9833 with reset"""
        print("Initializing AD9833...")
        self.write_ad9833(0x2100)  # Reset AD9833
        utime.sleep_ms(100)
        print("AD9833 initialized")
    
    def write_ad9833(self, data):
        """Write 16-bit data to AD9833 via SPI"""
        self.fsync.value(0)
        utime.sleep_us(1)
        
        data_bytes = data.to_bytes(2, 'big')
        self.spi.write(data_bytes)
        
        utime.sleep_us(1)
        self.fsync.value(1)
    
    def read_potentiometer(self):
        """Read potentiometer and convert to frequency"""
        pot_raw = self.freq_pot.read_u16()
        pot_value = pot_raw >> 4  # Convert to 12-bit (0-4095)
        
        # Map to logarithmic frequency scale
        frequency = self.map_logarithmic(pot_value, 0, 4095, MIN_FREQ, MAX_FREQ)
        
        return pot_value, frequency
    
    def map_logarithmic(self, input_val, in_min, in_max, out_min, out_max):
        """Map input to logarithmic output scale"""
        normalized = (input_val - in_min) / (in_max - in_min)
        log_min = math.log10(out_min)
        log_max = math.log10(out_max)
        log_value = log_min + normalized * (log_max - log_min)
        return 10 ** log_value
    
    def read_dip_switches(self):
        """Read DIP switch states"""
        sw1_state = not self.sw1.value()  # Inverted because of pullup
        sw2_state = not self.sw2.value()  # Inverted because of pullup
        return sw1_state, sw2_state
    
    def set_frequency_and_waveform(self, freq, sw1_state, sw2_state):
        """Set both frequency and waveform with complete reset - FIXED METHOD"""
        
        # Constrain frequency
        freq = max(MIN_FREQ, min(MAX_FREQ, freq))
        
        # Calculate frequency word
        freq_word = int((freq * 268435456.0) / 25000000.0)
        lsb = freq_word & 0x3FFF
        msb = (freq_word >> 14) & 0x3FFF
        
        # Determine waveform control register
        if not sw1_state and not sw2_state:
            waveform_name = "SINE"
            control_reg = 0x2000
            amplitude = "~0.6Vpp"
        elif not sw1_state and sw2_state:
            waveform_name = "TRIANGLE"
            control_reg = 0x2002
            amplitude = "~0.6Vpp"
        elif sw1_state and not sw2_state:
            waveform_name = "SQUARE"
            control_reg = 0x2028
            amplitude = "3.3Vpp"
        else:
            waveform_name = "SQUARE/2"
            control_reg = 0x2020
            amplitude = "3.3Vpp"
        
        # Complete setup sequence (this is the key fix!)
        self.write_ad9833(0x2100)           # Reset
        self.write_ad9833(lsb | 0x4000)     # FREQ0 LSB
        self.write_ad9833(msb | 0x4000)     # FREQ0 MSB
        self.write_ad9833(control_reg)      # Waveform selection
        
        return waveform_name, amplitude
    
    def format_frequency(self, frequency):
        """Format frequency for display"""
        if frequency >= 1000000:
            return f"{frequency/1000000:.3f} MHz"
        elif frequency >= 1000:
            return f"{frequency/1000:.2f} kHz"
        else:
            return f"{frequency:.1f} Hz"
    
    def format_switch_state(self, sw1_state, sw2_state):
        """Format switch states for display"""
        sw1_text = "ON " if sw1_state else "OFF"
        sw2_text = "ON " if sw2_state else "OFF"
        return f"SW1={sw1_text} SW2={sw2_text}"
    
    def detect_changes(self, pot_value, sw1_state, sw2_state):
        """Detect if any controls have changed significantly"""
        
        # Check potentiometer change (threshold to avoid noise)
        pot_changed = False
        if self.prev_pot_value is None:
            pot_changed = True
        else:
            # Consider changed if difference > 20 (about 0.5%)
            pot_changed = abs(pot_value - self.prev_pot_value) > 20
        
        # Check switch changes
        switch_changed = False
        if self.prev_sw1 is None or self.prev_sw2 is None:
            switch_changed = True
        else:
            switch_changed = (sw1_state != self.prev_sw1) or (sw2_state != self.prev_sw2)
        
        # Update previous values if changed
        if pot_changed:
            self.prev_pot_value = pot_value
        if switch_changed:
            self.prev_sw1 = sw1_state
            self.prev_sw2 = sw2_state
        
        return pot_changed, switch_changed
    
    def run_awg(self):
        """Main AWG operation loop"""
        print("\n=== AWG RUNNING ===")
        print("Turn potentiometer to change frequency")
        print("Flip DIP switches to change waveform")
        print("Press Ctrl+C to stop")
        print("==================")
        
        try:
            while True:
                # Read all inputs
                pot_value, frequency = self.read_potentiometer()
                sw1_state, sw2_state = self.read_dip_switches()
                
                # Check for changes
                pot_changed, switch_changed = self.detect_changes(pot_value, sw1_state, sw2_state)
                
                # Update if anything changed
                if pot_changed or switch_changed:
                    print(f"\n--- UPDATE ---")
                    if pot_changed:
                        print(f"Frequency: {self.format_frequency(frequency)} (Pot: {pot_value/4095*100:.1f}%)")
                    if switch_changed:
                        print(f"Switches: {self.format_switch_state(sw1_state, sw2_state)}")
                    
                    # Apply changes with complete reset
                    waveform_name, amplitude = self.set_frequency_and_waveform(frequency, sw1_state, sw2_state)
                    
                    print(f"Active: {self.format_frequency(frequency)} | {waveform_name} | {amplitude}")
                    print("-------------")
                
                # Brief status (less frequent)
                else:
                    # Show status every 50 cycles (about 5 seconds)
                    if hasattr(self, 'status_counter'):
                        self.status_counter += 1
                    else:
                        self.status_counter = 0
                    
                    if self.status_counter >= 50:
                        waveform_name, amplitude = self.get_waveform_info(sw1_state, sw2_state)
                        print(f"Status: {self.format_frequency(frequency)} | {waveform_name} | {amplitude}")
                        self.status_counter = 0
                
                utime.sleep(0.1)  # 10Hz update rate
                
        except KeyboardInterrupt:
            print("\n\nAWG stopped")
    
    def get_waveform_info(self, sw1_state, sw2_state):
        """Get waveform info without changing anything"""
        if not sw1_state and not sw2_state:
            return "SINE", "~0.6Vpp"
        elif not sw1_state and sw2_state:
            return "TRIANGLE", "~0.6Vpp"
        elif sw1_state and not sw2_state:
            return "SQUARE", "3.3Vpp"
        else:
            return "SQUARE/2", "3.3Vpp"

# Test functions
def test_frequency_range():
    """Test specific frequencies"""
    awg = AD9833_Working()
    
    test_frequencies = [1, 10, 100, 1000, 10000, 100000, 500000, 1000000]
    
    print("\n=== FREQUENCY RANGE TEST ===")
    
    try:
        for freq in test_frequencies:
            print(f"Testing {awg.format_frequency(freq)}...")
            awg.set_frequency_and_waveform(freq, False, False)  # Sine wave
            utime.sleep(2)
            
    except KeyboardInterrupt:
        print("Range test stopped")

# Main execution
if __name__ == "__main__":
    try:
        # Create and run AWG
        awg = AD9833_Working()
        awg.run_awg()
        
        # Alternative: Test frequency range
        # test_frequency_range()
        
    except Exception as e:
        print(f"Error: {e}")
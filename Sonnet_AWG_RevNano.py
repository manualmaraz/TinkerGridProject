import machine
import utime

# Pin definitions for Raspberry Pi Pico
FSYNC_PIN = 17      # GPIO17 - AD9833 Chip Select
SW1_PIN = 20        # GPIO20 - DIP Switch 1
SW2_PIN = 21        # GPIO21 - DIP Switch 2

# SPI pins (hardware SPI0)
SCK_PIN = 18        # GPIO18 - SPI Clock
MOSI_PIN = 19       # GPIO19 - SPI Data

class AD9833_DIP_Control:
    def __init__(self):
        # Initialize pins
        self.fsync = machine.Pin(FSYNC_PIN, machine.Pin.OUT)
        self.sw1 = machine.Pin(SW1_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
        self.sw2 = machine.Pin(SW2_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
        
        # Initialize SPI
        self.spi = machine.SPI(0, 
                              baudrate=1000000,
                              polarity=1, 
                              phase=1,
                              sck=machine.Pin(SCK_PIN),
                              mosi=machine.Pin(MOSI_PIN))
        
        print("=== AD9833 DIP Switch Control - MicroPython ===")
        print("Fixed Frequency: 1kHz")
        print("DIP Switch Waveform Selection:")
        print("")
        print("Waveform Selection Table:")
        print("SW1  SW2  | Waveform")
        print("OFF  OFF  | SINE")
        print("OFF  ON   | TRIANGLE")
        print("ON   OFF  | SQUARE")
        print("ON   ON   | SQUARE/2")
        print("=============================================")
        
        # Initialize AD9833
        self.init_ad9833()
        
        # Set fixed 1kHz frequency
        self.set_1khz_frequency()
        
        # Track previous switch state to detect changes
        self.prev_sw1 = None
        self.prev_sw2 = None
        
        print("Ready! Use DIP switches to change waveforms")
        print("Monitoring switch changes...")
    
    def init_ad9833(self):
        """Initialize AD9833 with reset"""
        print("\nInitializing AD9833...")
        self.write_ad9833(0x2100)  # Reset AD9833
        utime.sleep_ms(100)
        print("AD9833 reset complete")
    
    def write_ad9833(self, data):
        """Write 16-bit data to AD9833 via SPI"""
        self.fsync.value(0)  # Pull FSYNC low
        utime.sleep_us(1)
        
        # Send 16-bit data (MSB first)
        data_bytes = data.to_bytes(2, 'big')
        self.spi.write(data_bytes)
        
        utime.sleep_us(1)
        self.fsync.value(1)  # Pull FSYNC high
        
        # Debug: Show what was sent (only for important commands)
        if data == 0x2100 or (data & 0xF000) in [0x2000, 0x4000]:
            print(f"SPI Sent: 0x{data:04X}")
    
    def set_1khz_frequency(self):
        """Set fixed 1kHz frequency"""
        print("Setting fixed 1kHz frequency...")
        
        # Calculate frequency word for 1000Hz
        freq_word = int((1000.0 * 268435456.0) / 25000000.0)
        
        # Split into LSB and MSB (14 bits each)
        lsb = freq_word & 0x3FFF
        msb = (freq_word >> 14) & 0x3FFF
        
        # Send frequency data
        self.write_ad9833(0x2100)           # Reset
        self.write_ad9833(lsb | 0x4000)     # FREQ0 LSB
        self.write_ad9833(msb | 0x4000)     # FREQ0 MSB
        
        print("1kHz frequency set successfully")
    
    def read_dip_switches(self):
        """Read DIP switch states (inverted due to pullup)"""
        sw1_state = not self.sw1.value()  # Inverted because of pullup
        sw2_state = not self.sw2.value()  # Inverted because of pullup
        return sw1_state, sw2_state
    
    def set_waveform_from_switches(self, sw1_state, sw2_state):
        """Set waveform based on DIP switch combination"""
        
        # Determine waveform and control register
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
        else:  # sw1_state and sw2_state
            waveform_name = "SQUARE/2"
            control_reg = 0x2020
            amplitude = "3.3Vpp"
        
        # Send waveform command
        self.write_ad9833(control_reg)
        
        return waveform_name, amplitude
    
    def format_switch_state(self, sw1_state, sw2_state):
        """Format switch states for display"""
        sw1_text = "ON " if sw1_state else "OFF"
        sw2_text = "ON " if sw2_state else "OFF"
        return f"SW1={sw1_text} SW2={sw2_text}"
    
    def detect_switch_change(self, sw1_state, sw2_state):
        """Detect if switch state has changed"""
        if self.prev_sw1 is None or self.prev_sw2 is None:
            # First reading
            changed = True
        else:
            # Check for changes
            changed = (sw1_state != self.prev_sw1) or (sw2_state != self.prev_sw2)
        
        # Update previous state
        self.prev_sw1 = sw1_state
        self.prev_sw2 = sw2_state
        
        return changed
    
    def run_dip_control(self):
        """Main loop - monitor DIP switches and update waveform"""
        print("\n=== DIP Switch Control Active ===")
        print("Change DIP switches to select waveforms")
        print("Press Ctrl+C to stop")
        print("===================================")
        
        try:
            while True:
                # Read current switch states
                sw1_state, sw2_state = self.read_dip_switches()
                
                # Check if switches changed
                if self.detect_switch_change(sw1_state, sw2_state):
                    print(f"\n--- SWITCH CHANGE DETECTED ---")
                    print(f"DIP Switches: {self.format_switch_state(sw1_state, sw2_state)}")
                    
                    # Update waveform
                    waveform_name, amplitude = self.set_waveform_from_switches(sw1_state, sw2_state)
                    
                    print(f"Waveform: {waveform_name}")
                    print(f"Amplitude: {amplitude}")
                    print(f"Frequency: 1kHz (fixed)")
                    print("-----------------------------")
                
                # Show current status (less frequent)
                else:
                    # Get current waveform info
                    waveform_name, amplitude = self.get_current_waveform_info(sw1_state, sw2_state)
                    
                    print(f"Active: {waveform_name} | {amplitude} | 1kHz | "
                          f"{self.format_switch_state(sw1_state, sw2_state)}")
                
                utime.sleep(0.2)  # Check switches 5 times per second
                
        except KeyboardInterrupt:
            print("\n\nDIP switch control stopped")
            print("Current waveform remains active")
    
    def get_current_waveform_info(self, sw1_state, sw2_state):
        """Get current waveform information"""
        if not sw1_state and not sw2_state:
            return "SINE", "~0.6Vpp"
        elif not sw1_state and sw2_state:
            return "TRIANGLE", "~0.6Vpp"
        elif sw1_state and not sw2_state:
            return "SQUARE", "3.3Vpp"
        else:
            return "SQUARE/2", "3.3Vpp"
    
    def test_switch_reading(self):
        """Test DIP switch reading without changing waveforms"""
        print("\n=== DIP SWITCH TEST MODE ===")
        print("Testing switch reading only")
        print("No waveform changes will occur")
        print("Press Ctrl+C to stop")
        print("==============================")
        
        try:
            while True:
                sw1_state, sw2_state = self.read_dip_switches()
                
                print(f"Raw GPIO: SW1={self.sw1.value()} SW2={self.sw2.value()} | "
                      f"Processed: SW1={'ON' if sw1_state else 'OFF'} "
                      f"SW2={'ON' if sw2_state else 'OFF'} | "
                      f"Expected: {self.get_current_waveform_info(sw1_state, sw2_state)[0]}")
                
                utime.sleep(0.5)
                
        except KeyboardInterrupt:
            print("\nSwitch test stopped")

# Test functions
def test_all_waveforms_manual():
    """Manually test each waveform combination"""
    awg = AD9833_DIP_Control()
    
    test_combinations = [
        (False, False, "SINE"),
        (False, True, "TRIANGLE"),
        (True, False, "SQUARE"),
        (True, True, "SQUARE/2")
    ]
    
    print("\n=== MANUAL WAVEFORM TEST ===")
    print("Testing each waveform for 3 seconds...")
    
    try:
        for sw1, sw2, name in test_combinations:
            print(f"\nTesting: {name}")
            print(f"Simulating: SW1={'ON' if sw1 else 'OFF'}, SW2={'ON' if sw2 else 'OFF'}")
            
            waveform_name, amplitude = awg.set_waveform_from_switches(sw1, sw2)
            print(f"Set: {waveform_name} | {amplitude}")
            
            for i in range(3, 0, -1):
                print(f"  {name} active... {i}s remaining")
                utime.sleep(1)
                
    except KeyboardInterrupt:
        print("\nManual test stopped")

# Main execution
if __name__ == "__main__":
    try:
        # Create AWG instance
        awg = AD9833_DIP_Control()
        
        # Choose operation mode:
        awg.run_dip_control()              # Normal DIP switch control
        
        # Alternative test modes (uncomment to use):
        # awg.test_switch_reading()        # Test switch reading only
        # test_all_waveforms_manual()      # Manual waveform testing
        
    except Exception as e:
        print(f"Error: {e}")
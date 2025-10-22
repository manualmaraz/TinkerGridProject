# main.py - Raspberry Pi Pico Test Station (AWG + Multimeter)
# Stage-by-stage implementation for safe integration

from machine import Pin, I2C, ADC, SPI
import time
import math
from ssd1306 import SSD1306_I2C
from ina219 import INA219

# ============================================================================
# PIN ASSIGNMENTS - FINAL CONFIGURATION
# ============================================================================

# === AWG SECTION (ORIGINAL PINS - UNCHANGED) ===
FSYNC_PIN = 17          # AD9833 FSYNC
SPI_SCK_PIN = 18        # SPI Clock  
SPI_MOSI_PIN = 19       # SPI Data
DIP_SW1_PIN = 20        # Waveform DIP Switch 1
DIP_SW2_PIN = 21        # Waveform DIP Switch 2
FREQ_POT_PIN = 26       # Frequency potentiometer (ADC0)

# === MULTIMETER SECTION (REASSIGNED PINS) ===
I2C_SDA_PIN = 0         # I2C Data (OLED + INA219)
I2C_SCL_PIN = 1         # I2C Clock (OLED + INA219)
MULTIMETER_MODE_BUTTON_PIN = 16    # Multimeter mode cycling
BUZZER_PIN = 15         # Buzzer control (moved from 17)
RESISTANCE_ADC_PIN = 27 # Resistance measurement (moved from 26 to 27)

# === SYSTEM CONTROL ===
SYSTEM_MODE_BUTTON_PIN = 22  # Switch between AWG/Multimeter/Both modes

# ============================================================================
# HARDWARE CONFIGURATION
# ============================================================================

# I2C Configuration
I2C_BUS_ID = 0
OLED_WIDTH = 128
OLED_HEIGHT = 64
OLED_ADDR = 0x3C
INA219_ADDR = 0x40

# Multimeter Configuration
R_REF = 55000.0                    # Reference resistor (55kΩ)
V_SUPPLY = 3.3                     # Supply voltage
CONTINUITY_THRESHOLD_OHMS = 3.0    # Continuity threshold
OPEN_CIRCUIT_THRESHOLD_OHMS = 500000.0  # Open circuit threshold
NUM_ADC_SAMPLES = 100              # ADC averaging samples
DEBOUNCE_DELAY_MS = 200            # Button debounce delay

# AWG Configuration
MIN_FREQ = 1.0
MAX_FREQ = 1000000.0

# System Modes
SYSTEM_MODES = ["MULTIMETER", "AWG", "BOTH"]
MULTIMETER_MODES = [
    ("Bus Voltage", "V", 2),
    ("Current", "mA", 2), 
    ("Power", "mW", 2),
    ("Resistance", "Ohm", 1),
    ("Continuity", "", 0)
]

# ============================================================================
# MAIN TEST STATION CLASS
# ============================================================================

class PicoTestStation:
    def __init__(self, stage=1):
        """Initialize test station - stage parameter for gradual implementation"""
        self.stage = stage
        self.system_mode_index = 0  # Start with MULTIMETER
        self.multimeter_mode_index = 0
        self.last_system_button_press = 0
        self.last_multimeter_button_press = 0
        
        # AWG state
        self.current_frequency = 1000.0
        self.current_waveform = "SINE"
        self.awg_output_active = False
        self.prev_pot_value = None
        self.prev_sw1 = None
        self.prev_sw2 = None
        
        print(f"=== PICO TEST STATION - STAGE {stage} ===")
        print("Initializing hardware...")
        
        # Initialize based on stage
        if stage >= 1:
            self.init_basic_hardware()
        if stage >= 2:
            self.init_multimeter()
        if stage >= 3:
            self.init_awg()
        if stage >= 4:
            self.init_system_controls()
            
        print("Hardware initialization complete!")
        print(f"Stage {stage} ready for testing")
    
    # ========================================================================
    # STAGE 1: BASIC HARDWARE (I2C, OLED, INA219)
    # ========================================================================
    
    def init_basic_hardware(self):
        """Stage 1: Initialize I2C, OLED, and INA219"""
        print("Stage 1: Initializing I2C bus and displays...")
        
        try:
            # Initialize I2C
            self.i2c = I2C(I2C_BUS_ID, sda=Pin(I2C_SDA_PIN), scl=Pin(I2C_SCL_PIN), freq=400000)
            print(f"I2C initialized (SDA: GP{I2C_SDA_PIN}, SCL: GP{I2C_SCL_PIN})")
            
            # Scan I2C devices
            devices = self.i2c.scan()
            print(f"I2C devices found: {[hex(d) for d in devices]}")
            
            # Initialize OLED
            if OLED_ADDR in devices:
                self.oled = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, self.i2c, addr=OLED_ADDR)
                self.oled.fill(0)
                self.oled.text("Stage 1: I2C OK", 0, 0, 1)
                self.oled.text("OLED Working!", 0, 16, 1)
                self.oled.show()
                print("OLED initialized successfully")
            else:
                raise Exception(f"OLED not found at 0x{OLED_ADDR:02X}")
            
            # Initialize INA219
            if INA219_ADDR in devices:
                self.ina219 = INA219(self.i2c, addr=INA219_ADDR)
                print("INA219 initialized successfully")
            else:
                raise Exception(f"INA219 not found at 0x{INA219_ADDR:02X}")
                
        except Exception as e:
            print(f"Stage 1 initialization error: {e}")
            raise
    
    # ========================================================================
    # STAGE 2: MULTIMETER FUNCTIONALITY
    # ========================================================================
    
    def init_multimeter(self):
        """Stage 2: Initialize multimeter components"""
        print("Stage 2: Initializing multimeter components...")
        
        try:
            # Initialize multimeter-specific hardware
            self.resistance_adc = ADC(Pin(RESISTANCE_ADC_PIN))
            self.buzzer = Pin(BUZZER_PIN, Pin.OUT)
            self.buzzer.value(0)  # Ensure buzzer starts off
            self.multimeter_mode_button = Pin(MULTIMETER_MODE_BUTTON_PIN, Pin.IN, Pin.PULL_UP)
            
            print(f"Multimeter components initialized:")
            print(f"  - Resistance ADC: GP{RESISTANCE_ADC_PIN}")
            print(f"  - Buzzer: GP{BUZZER_PIN}")
            print(f"  - Mode button: GP{MULTIMETER_MODE_BUTTON_PIN}")
            
        except Exception as e:
            print(f"Stage 2 initialization error: {e}")
            raise
    
    # ========================================================================
    # STAGE 3: AWG FUNCTIONALITY  
    # ========================================================================
    
    def init_awg(self):
        """Stage 3: Initialize AWG components"""
        print("Stage 3: Initializing AWG components...")
        
        try:
            # Initialize AWG hardware
            self.fsync = Pin(FSYNC_PIN, Pin.OUT)
            self.spi = SPI(0, baudrate=1000000, polarity=1, phase=1,
                          sck=Pin(SPI_SCK_PIN), mosi=Pin(SPI_MOSI_PIN))
            self.freq_pot = ADC(Pin(FREQ_POT_PIN))
            self.dip_sw1 = Pin(DIP_SW1_PIN, Pin.IN, Pin.PULL_UP)
            self.dip_sw2 = Pin(DIP_SW2_PIN, Pin.IN, Pin.PULL_UP)
            
            # Initialize AD9833
            self.write_ad9833(0x2100)  # Reset
            time.sleep_ms(100)
            
            print(f"AWG components initialized:")
            print(f"  - AD9833 FSYNC: GP{FSYNC_PIN}")
            print(f"  - SPI: GP{SPI_SCK_PIN}/GP{SPI_MOSI_PIN}")
            print(f"  - Frequency pot: GP{FREQ_POT_PIN}")
            print(f"  - DIP switches: GP{DIP_SW1_PIN}/GP{DIP_SW2_PIN}")
            
        except Exception as e:
            print(f"Stage 3 initialization error: {e}")
            raise
    
    # ========================================================================
    # STAGE 4: SYSTEM INTEGRATION
    # ========================================================================
    
    def init_system_controls(self):
        """Stage 4: Initialize system-wide controls"""
        print("Stage 4: Initializing system controls...")
        
        try:
            self.system_mode_button = Pin(SYSTEM_MODE_BUTTON_PIN, Pin.IN, Pin.PULL_UP)
            print(f"System mode button: GP{SYSTEM_MODE_BUTTON_PIN}")
            
        except Exception as e:
            print(f"Stage 4 initialization error: {e}")
            raise
    
    # ========================================================================
    # MULTIMETER FUNCTIONS
    # ========================================================================
    
def read_multimeter_data(self):
    """Read all multimeter sensor data"""
    # INA219 readings using correct method names
    bus_voltage = self.ina219.get_bus_voltage()  # V
    current_A = self.ina219.get_current()        # A
    current_mA = current_A * 1000               # Convert to mA
    power_W = self.ina219.get_power()           # W
    power_mW = power_W * 1000                   # Convert to mW
    
    # Resistance measurement with averaging
    sum_adc = 0
    for _ in range(NUM_ADC_SAMPLES):
        sum_adc += self.resistance_adc.read_u16()
    raw_adc = sum_adc / NUM_ADC_SAMPLES
    v_out = (raw_adc / 65535.0) * V_SUPPLY
    
    # Calculate resistance
    if (V_SUPPLY - v_out) > 0.001 and v_out > 0.001:
        resistance_ohms = R_REF * (v_out / (V_SUPPLY - v_out))
    elif v_out >= (V_SUPPLY - 0.001):
        resistance_ohms = float('inf')
    else:
        resistance_ohms = 0.0
        
    return {
            'bus_voltage': bus_voltage,
            'current_mA': current_mA,
            'power_mW': power_mW,
            'resistance_ohms': resistance_ohms,
            'raw_adc': raw_adc,
            'v_out': v_out
    }
    
    def format_multimeter_display(self, data):
        """Format multimeter data for current mode"""
        mode_label, mode_unit, decimal_places = MULTIMETER_MODES[self.multimeter_mode_index]
        
        if mode_label == "Bus Voltage":
            value_str = f"{data['bus_voltage']:.{decimal_places}f}"
            unit_str = mode_unit
            buzzer_state = False
            
        elif mode_label == "Current":
            value_str = f"{data['current_mA']:.{decimal_places}f}"
            unit_str = mode_unit
            buzzer_state = False
            
        elif mode_label == "Power":
            value_str = f"{data['power_mW']:.{decimal_places}f}"
            unit_str = mode_unit
            buzzer_state = False
            
        elif mode_label == "Resistance":
            resistance = data['resistance_ohms']
            if resistance < 1000:
                value_str = f"{resistance:.{decimal_places}f}"
                unit_str = "Ohm"
            elif resistance < OPEN_CIRCUIT_THRESHOLD_OHMS:
                value_str = f"{resistance/1000:.{decimal_places}f}"
                unit_str = "kOhm"
            elif resistance >= OPEN_CIRCUIT_THRESHOLD_OHMS:
                value_str = "Open"
                unit_str = ""
            else:
                value_str = "Short"
                unit_str = ""
            buzzer_state = False
            
        elif mode_label == "Continuity":
            resistance = data['resistance_ohms']
            if resistance < CONTINUITY_THRESHOLD_OHMS:
                value_str = "Beep!"
                unit_str = ""
                buzzer_state = True
            elif resistance >= OPEN_CIRCUIT_THRESHOLD_OHMS:
                value_str = "Open"
                unit_str = ""
                buzzer_state = False
            else:
                value_str = "No Cont."
                unit_str = ""
                buzzer_state = False
        
        return {
            'mode': mode_label,
            'value': value_str,
            'unit': unit_str,
            'buzzer': buzzer_state
        }
    
    # ========================================================================
    # AWG FUNCTIONS
    # ========================================================================
    
    def write_ad9833(self, data):
        """Write data to AD9833"""
        self.fsync.value(0)
        time.sleep_us(1)
        data_bytes = data.to_bytes(2, 'big')
        self.spi.write(data_bytes)
        time.sleep_us(1)
        self.fsync.value(1)
    
    def read_awg_controls(self):
        """Read AWG control inputs"""
        # Read potentiometer
        pot_raw = self.freq_pot.read_u16()
        pot_value = pot_raw >> 4  # Convert to 12-bit
        frequency = self.map_logarithmic(pot_value, 0, 4095, MIN_FREQ, MAX_FREQ)
        
        # Read DIP switches
        sw1_state = not self.dip_sw1.value()
        sw2_state = not self.dip_sw2.value()
        
        # Determine waveform
        if not sw1_state and not sw2_state:
            waveform = "SINE"
        elif not sw1_state and sw2_state:
            waveform = "TRIANGLE"
        elif sw1_state and not sw2_state:
            waveform = "SQUARE"
        else:
            waveform = "SQUARE/2"
        
        return {
            'pot_value': pot_value,
            'frequency': frequency,
            'waveform': waveform,
            'sw1': sw1_state,
            'sw2': sw2_state
        }
    
    def map_logarithmic(self, input_val, in_min, in_max, out_min, out_max):
        """Map input to logarithmic scale"""
        normalized = (input_val - in_min) / (in_max - in_min)
        log_min = math.log10(out_min)
        log_max = math.log10(out_max)
        log_value = log_min + normalized * (log_max - log_min)
        return 10 ** log_value
    
    def set_awg_output(self, frequency, waveform):
        """Set AWG frequency and waveform"""
        # Constrain frequency
        frequency = max(MIN_FREQ, min(MAX_FREQ, frequency))
        
        # Calculate frequency word
        freq_word = int((frequency * 268435456.0) / 25000000.0)
        lsb = freq_word & 0x3FFF
        msb = (freq_word >> 14) & 0x3FFF
        
        # Waveform control registers
        waveform_regs = {
            "SINE": 0x2000,
            "TRIANGLE": 0x2002,
            "SQUARE": 0x2028,
            "SQUARE/2": 0x2020
        }
        
        control_reg = waveform_regs.get(waveform, 0x2000)
        
        # Send to AD9833
        self.write_ad9833(0x2100)           # Reset
        self.write_ad9833(lsb | 0x4000)     # FREQ0 LSB
        self.write_ad9833(msb | 0x4000)     # FREQ0 MSB
        self.write_ad9833(control_reg)      # Waveform
        
        # Update state
        self.current_frequency = frequency
        self.current_waveform = waveform
        self.awg_output_active = True
    
    def stop_awg_output(self):
        """Stop AWG output"""
        self.write_ad9833(0x2040)  # Sleep mode
        self.awg_output_active = False
    
    def format_frequency(self, frequency):
        """Format frequency for display"""
        if frequency >= 1000000:
            return f"{frequency/1000000:.2f}MHz"
        elif frequency >= 1000:
            return f"{frequency/1000:.1f}kHz"
        else:
            return f"{frequency:.0f}Hz"
    
    def detect_awg_changes(self, awg_data):
        """Detect if AWG controls have changed"""
        pot_changed = (self.prev_pot_value is None or 
                      abs(awg_data['pot_value'] - self.prev_pot_value) > 20)
        switch_changed = (self.prev_sw1 != awg_data['sw1'] or 
                         self.prev_sw2 != awg_data['sw2'])
        
        if pot_changed:
            self.prev_pot_value = awg_data['pot_value']
        if switch_changed:
            self.prev_sw1 = awg_data['sw1']
            self.prev_sw2 = awg_data['sw2']
        
        return pot_changed or switch_changed
    
    # ========================================================================
    # DISPLAY FUNCTIONS
    # ========================================================================
    
    def display_multimeter_screen(self, display_data):
        """Show full-screen multimeter display"""
        self.oled.fill(0)
        self.oled.text(display_data['mode'], 0, 0, 1)
        self.oled.text(display_data['value'], 0, 20, 2)  # Large font
        self.oled.text(display_data['unit'], 0, 48, 1)
        self.oled.show()
    
    def display_awg_screen(self, awg_data):
        """Show full-screen AWG display"""
        self.oled.fill(0)
        self.oled.text("AWG Generator", 0, 0, 1)
        self.oled.text(f"F: {self.format_frequency(awg_data['frequency'])}", 0, 16, 1)
        self.oled.text(f"W: {awg_data['waveform']}", 0, 32, 1)
        status = "ON" if self.awg_output_active else "OFF"
        self.oled.text(f"Out: {status}", 0, 48, 1)
        self.oled.show()
    
    def display_combined_screen(self, multimeter_data, awg_data):
        """Show split-screen display"""
        self.oled.fill(0)
        # Top half - AWG (32 pixels)
        self.oled.text("AWG:", 0, 0, 1)
        self.oled.text(f"{self.format_frequency(awg_data['frequency'])}", 0, 12, 1)
        self.oled.text(f"{awg_data['waveform']}", 64, 12, 1)
        
        # Bottom half - Multimeter (32 pixels)
        self.oled.text("METER:", 0, 32, 1)
        self.oled.text(f"{multimeter_data['mode'][:8]}", 0, 44, 1)
        self.oled.text(f"{multimeter_data['value'][:8]}", 0, 56, 1)
        self.oled.show()
    
    def display_stage_info(self):
        """Show current stage information"""
        self.oled.fill(0)
        self.oled.text(f"Test Station", 0, 0, 1)
        self.oled.text(f"Stage: {self.stage}", 0, 16, 1)
        self.oled.text(f"Mode: {SYSTEM_MODES[self.system_mode_index]}", 0, 32, 1)
        self.oled.text("Press buttons", 0, 48, 1)
        self.oled.show()
    
    # ========================================================================
    # BUTTON HANDLING
    # ========================================================================
    
    def check_buttons(self):
        """Check all button presses with debouncing"""
        current_time = time.ticks_ms()
        
        # System mode button (Stage 4+)
        if (self.stage >= 4 and 
            self.system_mode_button.value() == 0 and 
            time.ticks_diff(current_time, self.last_system_button_press) > DEBOUNCE_DELAY_MS):
            
            self.system_mode_index = (self.system_mode_index + 1) % len(SYSTEM_MODES)
            self.last_system_button_press = current_time
            print(f"System mode changed to: {SYSTEM_MODES[self.system_mode_index]}")
            
            # Wait for button release
            while self.system_mode_button.value() == 0:
                time.sleep_ms(10)
        
        # Multimeter mode button (Stage 2+)
        if (self.stage >= 2 and 
            self.multimeter_mode_button.value() == 0 and 
            time.ticks_diff(current_time, self.last_multimeter_button_press) > DEBOUNCE_DELAY_MS):
            
            self.multimeter_mode_index = (self.multimeter_mode_index + 1) % len(MULTIMETER_MODES)
            self.last_multimeter_button_press = current_time
            self.buzzer.value(0)  # Turn off buzzer on mode change
            print(f"Multimeter mode changed to: {MULTIMETER_MODES[self.multimeter_mode_index][0]}")
            
            # Wait for button release
            while self.multimeter_mode_button.value() == 0:
                time.sleep_ms(10)
    
    # ========================================================================
    # MAIN OPERATION MODES
    # ========================================================================
    
    def run_multimeter_mode(self):
        """Run in multimeter-only mode"""
        multimeter_data = self.read_multimeter_data()
        display_data = self.format_multimeter_display(multimeter_data)
        
        # Control buzzer
        self.buzzer.value(1 if display_data['buzzer'] else 0)
        
        # Update display
        self.display_multimeter_screen(display_data)
        
        # Debug output
        if self.stage <= 2:  # More verbose in early stages
            print(f"Multimeter: {display_data['mode']} = {display_data['value']} {display_data['unit']}")
    
    def run_awg_mode(self):
        """Run in AWG-only mode"""
        awg_data = self.read_awg_controls()
        
        # Update AWG if controls changed
        if self.detect_awg_changes(awg_data):
            self.set_awg_output(awg_data['frequency'], awg_data['waveform'])
            print(f"AWG: {self.format_frequency(awg_data['frequency'])} {awg_data['waveform']}")
        
        # Update display
        self.display_awg_screen(awg_data)
    
    def run_combined_mode(self):
        """Run both systems simultaneously"""
        # Read both systems
        multimeter_data = self.read_multimeter_data()
        multimeter_display = self.format_multimeter_display(multimeter_data)
        awg_data = self.read_awg_controls()
        
        # Update AWG if changed
        if self.detect_awg_changes(awg_data):
            self.set_awg_output(awg_data['frequency'], awg_data['waveform'])
        
        # Control buzzer
        self.buzzer.value(1 if multimeter_display['buzzer'] else 0)
        
        # Update display
        self.display_combined_screen(multimeter_display, awg_data)
    
    # ========================================================================
    # MAIN SYSTEM LOOP
    # ========================================================================
    
    def run_test_station(self):
        """Main system operation loop"""
        print(f"\n=== STARTING STAGE {self.stage} OPERATION ===")
        print("Press Ctrl+C to stop")
        
        if self.stage >= 4:
            print(f"System mode button: GP{SYSTEM_MODE_BUTTON_PIN}")
        if self.stage >= 2:
            print(f"Multimeter mode button: GP{MULTIMETER_MODE_BUTTON_PIN}")
        
        try:
            while True:
                # Check button presses
                self.check_buttons()
                
                # Run appropriate mode based on stage and selection
                if self.stage < 4:
                    # Early stages - run based on available hardware
                    if self.stage >= 3:
                        # Stage 3: Can run AWG or multimeter
                        if self.system_mode_index % 2 == 0:
                            self.run_multimeter_mode()
                        else:
                            self.run_awg_mode()
                    elif self.stage >= 2:
                        # Stage 2: Only multimeter
                        self.run_multimeter_mode()
                    else:
                        # Stage 1: Just display stage info
                        self.display_stage_info()
                else:
                    # Stage 4: Full system with mode switching
                    current_mode = SYSTEM_MODES[self.system_mode_index]
                    
                    if current_mode == "MULTIMETER":
                        self.run_multimeter_mode()
                    elif current_mode == "AWG":
                        self.run_awg_mode()
                    elif current_mode == "BOTH":
                        self.run_combined_mode()
                
                time.sleep_ms(100)  # 10Hz update rate
                
        except KeyboardInterrupt:
            print(f"\nStage {self.stage} testing stopped")
            if self.stage >= 2:
                self.buzzer.value(0)  # Turn off buzzer
            if self.stage >= 3:
                self.stop_awg_output()  # Stop AWG output

# ============================================================================
# STAGED TESTING FUNCTIONS
# ============================================================================

def test_stage_1():
    """Stage 1: Test I2C, OLED, and INA219 basic functionality"""
    print("\n" + "="*50)
    print("STAGE 1 TEST: Basic I2C and Display")
    print("="*50)
    print("This stage tests:")
    print("- I2C bus initialization")
    print("- OLED display functionality") 
    print("- INA219 sensor detection")
    print("- Basic voltage/current readings")
    print("\nExpected: OLED shows 'Stage 1: I2C OK' and sensor readings")
    print("="*50)
    
    station = PicoTestStation(stage=1)
    
    # Test basic sensor readings
    for i in range(10):
        try:
            voltage = station.ina219.get_bus_voltage()
            current_A = station.ina219.get_current()
            current_mA = current_A * 1000
            
            station.oled.fill(0)
            station.oled.text("Stage 1 Test", 0, 0, 1)
            station.oled.text(f"V: {voltage:.2f}V", 0, 16, 1)
            station.oled.text(f"I: {current_mA:.1f}mA", 0, 32, 1)
            station.oled.text(f"Count: {i+1}/10", 0, 48, 1)
            station.oled.show()
            
            print(f"Reading {i+1}: {voltage:.2f}V, {current_mA:.1f}mA")
            time.sleep(1)
            
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error in reading {i+1}: {e}")
            break
    
    print("Stage 1 test complete!")
    return station

def test_stage_2():
    """Stage 2: Add multimeter functionality"""
    print("\n" + "="*50)
    print("STAGE 2 TEST: Multimeter Functionality")
    print("="*50)
    print("This stage adds:")
    print(f"- Resistance measurement (GP{RESISTANCE_ADC_PIN})")
    print(f"- Buzzer control (GP{BUZZER_PIN})")
    print(f"- Mode button (GP{MULTIMETER_MODE_BUTTON_PIN})")
    print("- All 5 multimeter modes")
    print("\nExpected: Full multimeter with mode cycling")
    print("="*50)
    
    station = PicoTestStation(stage=2)
    station.run_test_station()
    return station

def test_stage_3():
    """Stage 3: Add AWG functionality"""
    print("\n" + "="*50)
    print("STAGE 3 TEST: AWG Integration")
    print("="*50)
    print("This stage adds:")
    print(f"- AD9833 DDS generator (GP{FSYNC_PIN})")
    print(f"- SPI communication (GP{SPI_SCK_PIN}, GP{SPI_MOSI_PIN})")
    print(f"- Frequency potentiometer (GP{FREQ_POT_PIN})")
    print(f"- Waveform DIP switches (GP{DIP_SW1_PIN}, GP{DIP_SW2_PIN})")
    print("- Manual switching between multimeter and AWG")
    print("\nExpected: Can switch between multimeter and AWG modes")
    print("="*50)
    
    station = PicoTestStation(stage=3)
    station.run_test_station()
    return station

def test_stage_4():
    """Stage 4: Full system integration"""
    print("\n" + "="*50)
    print("STAGE 4 TEST: Complete Test Station")
    print("="*50)
    print("This stage adds:")
    print(f"- System mode button (GP{SYSTEM_MODE_BUTTON_PIN})")
    print("- Three operation modes: MULTIMETER, AWG, BOTH")
    print("- Simultaneous operation capability")
    print("- Professional user interface")
    print("\nExpected: Full-featured test station")
    print("="*50)
    
    station = PicoTestStation(stage=4)
    station.run_test_station()
    return station

# ============================================================================
# MAIN EXECUTION
# ============================================================================

if __name__ == "__main__":
    print("RASPBERRY PI PICO TEST STATION")
    print("Staged Implementation for Safe Integration")
    print("\nAvailable stages:")
    print("1. Basic I2C and sensors")
    print("2. Full multimeter")
    print("3. Multimeter + AWG")
    print("4. Complete integrated system")
    
    # Uncomment the stage you want to test:
    
    test_stage_1()  # Start here for initial testing
    # test_stage_2()  # Add multimeter functionality
    # test_stage_3()  # Add AWG functionality
    #test_stage_4()    # Complete system (comment out for earlier stages)
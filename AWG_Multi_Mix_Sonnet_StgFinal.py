# main.py - Raspberry Pi Pico Test Station (AWG + Multimeter)
# Stage 4: Complete integrated system with DIP switch fix and welcome message
# FIXED: Manual waveform override now displays correctly on OLED

from machine import Pin, I2C, ADC, SPI
import time
import math
import select
import sys
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

# System Modes (Stage 4: All three modes available)
SYSTEM_MODES = ["MULTIMETER", "AWG", "BOTH"]
MULTIMETER_MODES = [
    ("Bus Voltage", "V", 2),
    ("Current", "mA", 2), 
    ("Power", "mW", 2),
    ("Resistance", "Ohm", 1),
    ("Continuity", "", 0)
]

# ============================================================================
# TERMINAL INPUT HANDLER
# ============================================================================

class TerminalHandler:
    """Handle non-blocking terminal input for menu control"""
    
    def __init__(self):
        self.input_buffer = ""
    
    def check_input(self):
        """Check for available terminal input (non-blocking)"""
        try:
            if select.select([sys.stdin], [], [], 0)[0]:
                char = sys.stdin.read(1)
                if char == '\n' or char == '\r':
                    command = self.input_buffer.strip().lower()
                    self.input_buffer = ""
                    return command
                else:
                    self.input_buffer += char
        except:
            pass
        return None

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
        self.menu_mode = True  # Enable menu control
        self.running = False
        
        # AWG state with manual frequency override
        self.current_frequency = 1000.0
        self.current_waveform = "SINE"
        self.awg_output_active = False
        self.prev_pot_value = None
        self.prev_sw1 = None
        self.prev_sw2 = None
        
        # Manual frequency override functionality
        self.manual_frequency_override = False  # Track if manual frequency is set
        self.manual_frequency_value = 1000.0    # Store manual frequency
        
        # FIXED: Manual waveform override functionality
        self.manual_waveform_override = False   # Track if manual waveform is set
        self.manual_waveform_value = "SINE"     # Store manual waveform
        
        # Terminal handler
        self.terminal = TerminalHandler()
        
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
    # STAGE INITIALIZATION
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
                
                # Show welcome message
                self.show_welcome_message()
                
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
    
    def show_welcome_message(self):
        """Display welcome message on OLED"""
        self.oled.fill(0)
        
        # Center the text on screen
        self.oled.text("TinkerGrid", 25, 16, 1)
        self.oled.text("Telemetry", 25, 32, 1)
        self.oled.text("System", 35, 48, 1)
        
        self.oled.show()
        time.sleep(2)  # Show welcome message for 2 seconds
        
        # Clear screen after welcome
        self.oled.fill(0)
        self.oled.text("Initializing...", 10, 28, 1)
        self.oled.show()
    
    def init_multimeter(self):
        """Stage 2: Initialize multimeter components"""
        print("Stage 2: Initializing multimeter components...")
        
        try:
            # Initialize multimeter-specific hardware
            self.resistance_adc = ADC(Pin(RESISTANCE_ADC_PIN))
            self.buzzer = Pin(BUZZER_PIN, Pin.OUT)
            self.buzzer.value(0)  # Ensure buzzer starts off
            
            print(f"Multimeter components initialized:")
            print(f"  - Resistance ADC: GP{RESISTANCE_ADC_PIN}")
            print(f"  - Buzzer: GP{BUZZER_PIN}")
            print(f"  - Mode control: Terminal menu")
            
        except Exception as e:
            print(f"Stage 2 initialization error: {e}")
            raise
    
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
            
            # Initialize DIP switch state tracking
            self.prev_sw1 = not self.dip_sw1.value()
            self.prev_sw2 = not self.dip_sw2.value()
            
            # Initialize AD9833
            self.write_ad9833(0x2100)  # Reset
            time.sleep_ms(100)
            
            print(f"AWG components initialized:")
            print(f"  - AD9833 FSYNC: GP{FSYNC_PIN}")
            print(f"  - SPI: GP{SPI_SCK_PIN}/GP{SPI_MOSI_PIN}")
            print(f"  - Frequency pot: GP{FREQ_POT_PIN}")
            print(f"  - DIP switches: GP{DIP_SW1_PIN}/GP{DIP_SW2_PIN}")
            print(f"  - Frequency control: Potentiometer + Manual override")
            print(f"  - Waveform control: DIP switches + Manual override")
            print(f"  - DIP switch refresh: Enabled")
            
        except Exception as e:
            print(f"Stage 3 initialization error: {e}")
            raise
    
    def init_system_controls(self):
        """Stage 4: Initialize system-wide controls"""
        print("Stage 4: Initializing complete system integration...")
        print(f"System features:")
        print(f"  - Three operation modes: MULTIMETER, AWG, BOTH")
        print(f"  - Simultaneous operation capability")
        print(f"  - Professional terminal interface")
        print(f"  - Complete test station functionality")
        print(f"  - Real-time DIP switch and potentiometer updates")
        print(f"  - Manual frequency and waveform override")
    
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
    # AWG FUNCTIONS WITH ENHANCED DIP SWITCH DETECTION AND WAVEFORM OVERRIDE
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
        """Read AWG control inputs with manual frequency and waveform override"""
        # Check if manual frequency override is active
        if self.manual_frequency_override:
            frequency = self.manual_frequency_value
            pot_value = 0  # Not used when manual override is active
        else:
            # Read potentiometer for frequency
            pot_raw = self.freq_pot.read_u16()
            pot_value = pot_raw >> 4  # Convert to 12-bit
            frequency = self.map_logarithmic(pot_value, 0, 4095, MIN_FREQ, MAX_FREQ)
        
        # Check if manual waveform override is active
        if self.manual_waveform_override:
            waveform = self.manual_waveform_value
            # Still read DIP switches for state tracking, but don't use for waveform
            sw1_state = not self.dip_sw1.value()
            sw2_state = not self.dip_sw2.value()
        else:
            # Read DIP switches for waveform
            sw1_state = not self.dip_sw1.value()
            sw2_state = not self.dip_sw2.value()
            
            # Determine waveform from DIP switches
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
            'sw2': sw2_state,
            'manual_frequency_override': self.manual_frequency_override,
            'manual_waveform_override': self.manual_waveform_override
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
        """Enhanced AWG change detection with DIP switch refresh fix"""
        # Only check potentiometer if manual override is not active
        pot_changed = (not self.manual_frequency_override and 
                      self.prev_pot_value is not None and 
                      abs(awg_data['pot_value'] - self.prev_pot_value) > 20)
        
        # Only check DIP switches if manual waveform override is not active
        switch_changed = (not self.manual_waveform_override and
                         (self.prev_sw1 != awg_data['sw1'] or 
                          self.prev_sw2 != awg_data['sw2']))
        
        # Update previous values
        if not self.manual_frequency_override and pot_changed:
            self.prev_pot_value = awg_data['pot_value']
        
        # Always update DIP switch state for tracking
        if switch_changed:
            self.prev_sw1 = awg_data['sw1']
            self.prev_sw2 = awg_data['sw2']
            print(f"DIP switch changed: SW1={awg_data['sw1']}, SW2={awg_data['sw2']} → {awg_data['waveform']}")
        
        return pot_changed or switch_changed
    
    # ========================================================================
    # DISPLAY FUNCTIONS WITH STAGE 4 ENHANCEMENTS AND WAVEFORM OVERRIDE FIX
    # ========================================================================
    
    def display_multimeter_screen(self, display_data):
        """Show full-screen multimeter display"""
        self.oled.fill(0)
        self.oled.text(display_data['mode'], 0, 0, 1)
        self.oled.text(display_data['value'], 0, 20, 1)
        self.oled.text(display_data['unit'], 0, 48, 1)
        self.oled.show()
    
    def display_awg_screen(self, awg_data):
        """Show full-screen AWG display with override indicators"""
        self.oled.fill(0)
        self.oled.text("AWG Generator", 0, 0, 1)
        
        # Show frequency with override indicator
        freq_indicator = "M" if awg_data.get('manual_frequency_override', False) else "P"
        self.oled.text(f"F: {self.format_frequency(awg_data['frequency'])} {freq_indicator}", 0, 16, 1)
        
        # Show waveform with override indicator
        wave_indicator = "M" if awg_data.get('manual_waveform_override', False) else "D"
        self.oled.text(f"W: {awg_data['waveform']} {wave_indicator}", 0, 32, 1)
        
        status = "ON" if self.awg_output_active else "OFF"
        self.oled.text(f"Out: {status}", 0, 48, 1)
        self.oled.show()
    
    def display_combined_screen(self, multimeter_data, awg_data):
        """Show split-screen display - STAGE 4 FEATURE"""
        self.oled.fill(0)
        
        # Top half - AWG (32 pixels)
        self.oled.text("AWG:", 0, 0, 1)
        freq_indicator = "M" if awg_data.get('manual_frequency_override', False) else "P"
        freq_text = f"{self.format_frequency(awg_data['frequency'])}{freq_indicator}"
        # Truncate if too long
        if len(freq_text) > 10:
            freq_text = freq_text[:10]
        self.oled.text(freq_text, 0, 12, 1)
        
        # Show waveform with override indicator
        wave_indicator = "M" if awg_data.get('manual_waveform_override', False) else "D"
        waveform_short = awg_data['waveform'][:6]  # Shorter to fit indicator
        status = "ON" if self.awg_output_active else "OFF"
        self.oled.text(f"{waveform_short}{wave_indicator} {status}", 64, 12, 1)
        
        # Divider line
        for x in range(128):
            self.oled.pixel(x, 24, 1)
        
        # Bottom half - Multimeter (32 pixels)
        self.oled.text("METER:", 0, 32, 1)
        mode_short = multimeter_data['mode'][:8]  # Truncate mode name
        self.oled.text(mode_short, 0, 44, 1)
        
        value_unit = f"{multimeter_data['value']} {multimeter_data['unit']}"
        if len(value_unit) > 16:
            value_unit = value_unit[:16]
        self.oled.text(value_unit, 0, 56, 1)
        
        self.oled.show()
    
    def display_stage_info(self):
        """Show current stage information"""
        self.oled.fill(0)
        self.oled.text(f"TinkerGrid", 25, 0, 1)
        self.oled.text(f"Test Station", 15, 16, 1)
        self.oled.text(f"Stage: {self.stage}", 0, 32, 1)
        self.oled.text(f"Mode: {SYSTEM_MODES[self.system_mode_index]}", 0, 48, 1)
        self.oled.show()
    
    # ========================================================================
    # TERMINAL MENU SYSTEM - STAGE 4 COMPLETE WITH WAVEFORM OVERRIDE
    # ========================================================================
    
    def show_main_menu(self):
        """Display main menu options - Stage 4 complete version with waveform override"""
        print("\n" + "="*70)
        print("TINKERGRID TELEMETRY SYSTEM - COMPLETE (STAGE 4)")
        print("="*70)
        print(f"Current Stage: {self.stage}")
        print(f"System Mode: {SYSTEM_MODES[self.system_mode_index]}")
        if self.stage >= 2:
            print(f"Multimeter Mode: {MULTIMETER_MODES[self.multimeter_mode_index][0]}")
        if self.stage >= 3:
            print(f"AWG Status: {'ON' if self.awg_output_active else 'OFF'}")
            print(f"AWG Frequency: {self.format_frequency(self.current_frequency)}")
            print(f"AWG Waveform: {self.current_waveform}")
            freq_control = "Manual Override" if self.manual_frequency_override else "Potentiometer"
            wave_control = "Manual Override" if self.manual_waveform_override else "DIP Switches"
            print(f"Frequency Control: {freq_control}")
            print(f"Waveform Control: {wave_control}")
        print("="*70)
        
        print("\nSYSTEM COMMANDS:")
        if self.stage >= 4:
            print("  s - Switch system mode (MULTIMETER → AWG → BOTH)")
        elif self.stage >= 3:
            print("  s - Switch system mode (MULTIMETER ↔ AWG)")
        print("  r - Start/stop continuous readings")
        print("  q - Quit")
        
        if self.stage >= 2:
            print("\nMULTIMETER COMMANDS:")
            print("  m - Change multimeter mode")
            print("  v - Show voltage reading")
            print("  i - Show current reading")
            print("  p - Show power reading")
            print("  o - Show resistance reading")
            print("  c - Show continuity test")
        
        if self.stage >= 3:
            print("\nAWG COMMANDS:")
            print("  a - Toggle AWG output ON/OFF")
            print("  f - Set specific frequency (overrides potentiometer)")
            print("  fp - Return frequency control to potentiometer")
            print("  w - Set waveform (overrides DIP switches)")
            print("  wd - Return waveform control to DIP switches")
            print("  awg - Switch to AWG mode")
            print("  meter - Switch to multimeter mode")
            if self.stage >= 4:
                print("  both - Switch to simultaneous mode")
        
        print("\nSTATUS COMMANDS:")
        print("  h - Show this help menu")
        print("  t - Show current readings")
        if self.stage >= 4:
            print("  demo - Run demonstration sequence")
        print("="*70)
    
    def handle_menu_command(self, command):
        """Handle terminal menu commands - Stage 4 complete version with waveform override"""
        if command == 'h':
            self.show_main_menu()
            
        elif command == 'q':
            print("Shutting down TinkerGrid Telemetry System...")
            self.running = False
            if self.stage >= 2:
                self.buzzer.value(0)
            if self.stage >= 3:
                self.stop_awg_output()
            return False
            
        elif command == 'r':
            if not self.running:
                print("Starting continuous readings... (Press 'q' to stop)")
                self.running = True
            else:
                print("Stopping continuous readings...")
                self.running = False
                
        elif command == 's' and self.stage >= 3:
            # Stage 4: Cycle through all three modes
            if self.stage >= 4:
                self.system_mode_index = (self.system_mode_index + 1) % 3
            else:
                # Stage 3: Only toggle between MULTIMETER and AWG
                self.system_mode_index = (self.system_mode_index + 1) % 2
            print(f"System mode changed to: {SYSTEM_MODES[self.system_mode_index]}")
            
        elif command == 'awg' and self.stage >= 3:
            self.system_mode_index = 1  # AWG mode
            print("Switched to AWG mode")
            
        elif command == 'meter' and self.stage >= 3:
            self.system_mode_index = 0  # Multimeter mode
            print("Switched to Multimeter mode")
            
        elif command == 'both' and self.stage >= 4:
            self.system_mode_index = 2  # BOTH mode
            print("Switched to simultaneous operation mode")
            
        elif command == 'm' and self.stage >= 2:
            self.multimeter_mode_index = (self.multimeter_mode_index + 1) % len(MULTIMETER_MODES)
            mode_name = MULTIMETER_MODES[self.multimeter_mode_index][0]
            print(f"Multimeter mode changed to: {mode_name}")
            self.buzzer.value(0)  # Turn off buzzer on mode change
            
        elif command == 't':
            self.show_current_readings()
            
        elif command == 'demo' and self.stage >= 4:
            self.run_demonstration()
            
        # Direct multimeter mode commands
        elif command == 'v' and self.stage >= 2:
            self.multimeter_mode_index = 0  # Bus Voltage
            print("Switched to Bus Voltage mode")
            
        elif command == 'i' and self.stage >= 2:
            self.multimeter_mode_index = 1  # Current
            print("Switched to Current mode")
            
        elif command == 'p' and self.stage >= 2:
            self.multimeter_mode_index = 2  # Power
            print("Switched to Power mode")
            
        elif command == 'o' and self.stage >= 2:
            self.multimeter_mode_index = 3  # Resistance
            print("Switched to Resistance mode")
            
        elif command == 'c' and self.stage >= 2:
            self.multimeter_mode_index = 4  # Continuity
            print("Switched to Continuity mode")
        
        # AWG commands with manual frequency and waveform override
        elif command == 'a' and self.stage >= 3:
            if self.awg_output_active:
                self.stop_awg_output()
                print("AWG output stopped")
            else:
                awg_data = self.read_awg_controls()
                self.set_awg_output(awg_data['frequency'], awg_data['waveform'])
                freq_source = "manual override" if self.manual_frequency_override else "potentiometer"
                wave_source = "manual override" if self.manual_waveform_override else "DIP switches"
                print(f"AWG output started: {self.format_frequency(self.current_frequency)} {self.current_waveform}")
                print(f"  Frequency source: {freq_source}")
                print(f"  Waveform source: {wave_source}")
                
        elif command == 'f' and self.stage >= 3:
            try:
                freq_str = input("Enter frequency (Hz): ")
                frequency = float(freq_str)
                frequency = max(MIN_FREQ, min(MAX_FREQ, frequency))
                
                # Enable manual frequency override
                self.manual_frequency_override = True
                self.manual_frequency_value = frequency
                
                awg_data = self.read_awg_controls()
                self.set_awg_output(frequency, awg_data['waveform'])
                print(f"Manual frequency set to: {self.format_frequency(frequency)} (overrides potentiometer)")
            except ValueError:
                print("Invalid frequency value")
                
        elif command == 'fp' and self.stage >= 3:
            self.manual_frequency_override = False
            print("Frequency control returned to potentiometer")
            
        elif command == 'w' and self.stage >= 3:
            waveforms = ["SINE", "TRIANGLE", "SQUARE", "SQUARE/2"]
            print("Available waveforms:")
            for i, wf in enumerate(waveforms):
                print(f"  {i+1} - {wf}")
            try:
                choice = int(input("Select waveform (1-4): ")) - 1
                if 0 <= choice < len(waveforms):
                    waveform = waveforms[choice]
                    
                    # Enable manual waveform override
                    self.manual_waveform_override = True
                    self.manual_waveform_value = waveform
                    
                    awg_data = self.read_awg_controls()
                    self.set_awg_output(awg_data['frequency'], waveform)
                    print(f"Manual waveform set to: {waveform} (overrides DIP switches)")
                else:
                    print("Invalid waveform selection")
            except ValueError:
                print("Invalid selection")
                
        elif command == 'wd' and self.stage >= 3:
            self.manual_waveform_override = False
            print("Waveform control returned to DIP switches")
                
        else:
            print(f"Unknown command: '{command}'. Type 'h' for help.")
        
        return True
    
    def show_current_readings(self):
        """Show current sensor readings - Stage 4 enhanced version with waveform override"""
        print("\n" + "-"*70)
        print("CURRENT READINGS - TINKERGRID TELEMETRY SYSTEM:")
        print("-"*70)
        
        if self.stage >= 2:
            try:
                data = self.read_multimeter_data()
                display_data = self.format_multimeter_display(data)
                
                print("MULTIMETER SECTION:")
                print(f"  Bus Voltage: {data['bus_voltage']:.2f} V")
                print(f"  Current: {data['current_mA']:.2f} mA")
                print(f"  Power: {data['power_mW']:.2f} mW")
                
                resistance = data['resistance_ohms']
                if resistance < 1000:
                    print(f"  Resistance: {resistance:.1f} Ω")
                elif resistance < OPEN_CIRCUIT_THRESHOLD_OHMS:
                    print(f"  Resistance: {resistance/1000:.1f} kΩ")
                else:
                    print("  Resistance: Open Circuit")
                
                print(f"  Active Mode: {display_data['mode']} = {display_data['value']} {display_data['unit']}")
                
            except Exception as e:
                print(f"Error reading multimeter: {e}")
        
        if self.stage >= 3:
            try:
                awg_data = self.read_awg_controls()
                print("\nAWG SECTION:")
                freq_source = "Manual Override" if awg_data['manual_frequency_override'] else f"Potentiometer (ADC: {awg_data['pot_value']})"
                wave_source = "Manual Override" if awg_data['manual_waveform_override'] else f"DIP Switches (SW1: {awg_data['sw1']}, SW2: {awg_data['sw2']})"
                print(f"  Frequency: {self.format_frequency(awg_data['frequency'])} ({freq_source})")
                print(f"  Waveform: {awg_data['waveform']} ({wave_source})")
                print(f"  Output: {'ON' if self.awg_output_active else 'OFF'}")
            except Exception as e:
                print(f"Error reading AWG: {e}")
        
        if self.stage >= 4:
            print(f"\nSYSTEM STATUS:")
            print(f"  Operation Mode: {SYSTEM_MODES[self.system_mode_index]}")
            print(f"  Continuous Readings: {'ACTIVE' if self.running else 'STOPPED'}")
            print(f"  Stage: {self.stage}/4 (Complete)")
            print(f"  Manual Overrides: Frequency={'ON' if self.manual_frequency_override else 'OFF'}, Waveform={'ON' if self.manual_waveform_override else 'OFF'}")
        
        print("-"*70)
    
    def run_demonstration(self):
        """Run a demonstration sequence - Stage 4 feature"""
        print("\n" + "="*60)
        print("RUNNING TINKERGRID TELEMETRY SYSTEM DEMONSTRATION")
        print("="*60)
        
        original_mode = self.system_mode_index
        original_running = self.running
        
        try:
            # Demo sequence
            demo_steps = [
                ("MULTIMETER", 0, "Demonstrating multimeter functionality"),
                ("AWG", 1, "Demonstrating AWG functionality"),
                ("BOTH", 2, "Demonstrating simultaneous operation")
            ]
            
            for mode_name, mode_index, description in demo_steps:
                print(f"\n--- {description} ---")
                self.system_mode_index = mode_index
                self.running = True
                
                # Run for 5 seconds
                for i in range(50):  # 5 seconds at 10Hz
                    if mode_name == "MULTIMETER":
                        self.run_multimeter_mode()
                    elif mode_name == "AWG":
                        self.run_awg_mode()
                    elif mode_name == "BOTH":
                        self.run_combined_mode()
                    
                    time.sleep_ms(100)
                
                self.running = False
                print(f"{mode_name} mode demonstration complete")
                time.sleep(1)
            
            print("\n" + "="*60)
            print("DEMONSTRATION COMPLETE")
            print("TinkerGrid Telemetry System - All systems functioning correctly!")
            print("="*60)
            
        except Exception as e:
            print(f"Demonstration error: {e}")
        finally:
            # Restore original state
            self.system_mode_index = original_mode
            self.running = original_running
    
    # ========================================================================
    # MAIN OPERATION MODES - STAGE 4 COMPLETE WITH WAVEFORM OVERRIDE
    # ========================================================================
    
    def run_multimeter_mode(self):
        """Run in multimeter-only mode"""
        multimeter_data = self.read_multimeter_data()
        display_data = self.format_multimeter_display(multimeter_data)
        
        # Control buzzer
        self.buzzer.value(1 if display_data['buzzer'] else 0)
        
        # Update display
        self.display_multimeter_screen(display_data)
        
        return display_data
    
    def run_awg_mode(self):
        """Run in AWG-only mode"""
        awg_data = self.read_awg_controls()
        
        # Update AWG if controls changed (respects manual overrides)
        if self.detect_awg_changes(awg_data):
            if self.awg_output_active:
                self.set_awg_output(awg_data['frequency'], awg_data['waveform'])
                freq_source = "manual" if awg_data['manual_frequency_override'] else "potentiometer"
                wave_source = "manual" if awg_data['manual_waveform_override'] else "DIP switches"
                print(f"AWG updated: {self.format_frequency(awg_data['frequency'])} {awg_data['waveform']} (F:{freq_source}, W:{wave_source})")
        
        # Update display (will show correct waveform with override indicators)
        self.display_awg_screen(awg_data)
        
        return awg_data
    
    def run_combined_mode(self):
        """Run both systems simultaneously - STAGE 4 MAIN FEATURE"""
        # Read both systems
        multimeter_data = self.read_multimeter_data()
        multimeter_display = self.format_multimeter_display(multimeter_data)
        awg_data = self.read_awg_controls()
        
        # Update AWG if changed (respects manual overrides)
        if self.detect_awg_changes(awg_data):
            if self.awg_output_active:
                self.set_awg_output(awg_data['frequency'], awg_data['waveform'])
        
        # Control buzzer
        self.buzzer.value(1 if multimeter_display['buzzer'] else 0)
        
        # Update display with split-screen (will show correct waveform with indicators)
        self.display_combined_screen(multimeter_display, awg_data)
        
        return multimeter_display, awg_data
    
    # ========================================================================
    # MAIN SYSTEM LOOP - STAGE 4 COMPLETE
    # ========================================================================
    
    def run_test_station_with_menu(self):
        """Main system operation loop - Stage 4 complete version"""
        print(f"\n=== STARTING TINKERGRID TELEMETRY SYSTEM (STAGE {self.stage}) ===")
        self.show_main_menu()
        
        try:
            while True:
                # Check for terminal commands
                command = self.terminal.check_input()
                if command is not None:
                    if not self.handle_menu_command(command):
                        break  # Quit command received
                
                # Run continuous readings if enabled
                if self.running:
                    try:
                        # Stage 4: Full system with all three modes
                        current_mode = SYSTEM_MODES[self.system_mode_index]
                        
                        if current_mode == "MULTIMETER":
                            self.run_multimeter_mode()
                        elif current_mode == "AWG":
                            self.run_awg_mode()
                        elif current_mode == "BOTH":
                            self.run_combined_mode()
                    
                    except Exception as e:
                        print(f"Error in operation: {e}")
                        self.running = False
                
                time.sleep_ms(100)  # 10Hz update rate
                
        except KeyboardInterrupt:
            print(f"\nTinkerGrid Telemetry System interrupted")
        finally:
            if self.stage >= 2:
                self.buzzer.value(0)  # Turn off buzzer
            if self.stage >= 3:
                self.stop_awg_output()  # Stop AWG output

# ============================================================================
# COMPLETE TESTING FUNCTIONS
# ============================================================================

def test_stage_4_complete():
    """Stage 4: Complete TinkerGrid Telemetry System with Waveform Override Fix"""
    print("\n" + "="*70)
    print("STAGE 4: TINKERGRID TELEMETRY SYSTEM - COMPLETE + WAVEFORM FIX")
    print("="*70)
    print("Final system with all features and fixes:")
    print(f"- AD9833 DDS generator (GP{FSYNC_PIN})")
    print(f"- SPI communication (GP{SPI_SCK_PIN}, GP{SPI_MOSI_PIN})")
    print(f"- Frequency potentiometer (GP{FREQ_POT_PIN}) + manual override")
    print(f"- Waveform DIP switches (GP{DIP_SW1_PIN}, GP{DIP_SW2_PIN}) + manual override")
    print(f"- Complete multimeter (GP{RESISTANCE_ADC_PIN}, GP{BUZZER_PIN})")
    print("- Three operation modes: MULTIMETER, AWG, BOTH")
    print("- Simultaneous operation capability")
    print("- Split-screen display in BOTH mode")
    print("- Professional terminal interface")
    print("- Welcome message: 'TinkerGrid Telemetry System'")
    print("- Real-time DIP switch refresh fix")
    print("- Manual waveform override with OLED display fix")
    print("- Demonstration mode")
    print("\nFIXES APPLIED:")
    print("✅ DIP switch changes now refresh OLED display immediately")
    print("✅ Welcome message shows 'TinkerGrid Telemetry System'")
    print("✅ Enhanced change detection for all controls")
    print("✅ Manual waveform override now displays correctly on OLED")
    print("✅ Override indicators: M=Manual, P=Potentiometer, D=DIP switches")
    print("✅ 'wd' command to return waveform control to DIP switches")
    print("\nThis is the complete, professional-grade telemetry system!")
    print("="*70)
    
    station = PicoTestStation(stage=4)
    station.run_test_station_with_menu()
    return station

# ============================================================================
# MAIN EXECUTION
# ============================================================================

if __name__ == "__main__":
    print("TINKERGRID TELEMETRY SYSTEM")
    print("COMPLETE IMPLEMENTATION - STAGE 4 + WAVEFORM OVERRIDE FIX")
    print("Professional AWG + Multimeter Test Station")
    print("\nFinal Hardware Configuration:")
    print("- GP0/GP1: I2C (OLED + INA219)")
    print("- GP15: Buzzer")
    print("- GP17: AD9833 FSYNC")
    print("- GP18/GP19: SPI (AD9833)")
    print("- GP20/GP21: DIP switches (waveform) - REAL-TIME REFRESH + MANUAL OVERRIDE")
    print("- GP26: Frequency potentiometer + MANUAL OVERRIDE")
    print("- GP27: Resistance measurement")
    print("\nFeatures:")
    print("- Complete multimeter (5 modes)")
    print("- Professional AWG (1Hz-1MHz)")
    print("- Simultaneous operation")
    print("- Manual frequency override ('f' command, 'fp' to return)")
    print("- Manual waveform override ('w' command, 'wd' to return)")
    print("- Real-time DIP switch updates")
    print("- Split-screen display")
    print("- Terminal menu control")
    print("- Welcome message")
    print("- Demonstration mode")
    print("- Override indicators on OLED (M/P/D)")
    
    # Run the complete Stage 4 TinkerGrid Telemetry System with waveform fix
    test_stage_4_complete()
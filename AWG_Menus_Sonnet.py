import machine
import utime
import math
import sys

# Pin definitions
FSYNC_PIN = 17
SW1_PIN = 20
SW2_PIN = 21
FREQ_POT_PIN = 26
SCK_PIN = 18
MOSI_PIN = 19

MIN_FREQ = 1.0
MAX_FREQ = 1000000.0

class AD9833_MenuDriven:
    def __init__(self):
        # Initialize hardware
        self.fsync = machine.Pin(FSYNC_PIN, machine.Pin.OUT)
        self.sw1 = machine.Pin(SW1_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
        self.sw2 = machine.Pin(SW2_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
        self.freq_pot = machine.ADC(FREQ_POT_PIN)
        
        self.spi = machine.SPI(0, 
                              baudrate=1000000,
                              polarity=1, 
                              phase=1,
                              sck=machine.Pin(SCK_PIN),
                              mosi=machine.Pin(MOSI_PIN))
        
        # Current settings
        self.current_frequency = 1000.0
        self.current_waveform = "SINE"
        self.output_active = False
        
        # Manual mode state tracking
        self.prev_pot_value = None
        self.prev_sw1 = None
        self.prev_sw2 = None
        
        # Initialize AD9833
        self.init_ad9833()
        
        print("*** AD9833 Professional AWG System ***")
        print("=" * 40)
    
    def init_ad9833(self):
        """Initialize AD9833"""
        self.write_ad9833(0x2100)  # Reset
        utime.sleep_ms(100)
    
    def write_ad9833(self, data):
        """Write to AD9833"""
        self.fsync.value(0)
        utime.sleep_us(1)
        data_bytes = data.to_bytes(2, 'big')
        self.spi.write(data_bytes)
        utime.sleep_us(1)
        self.fsync.value(1)
    
    def set_frequency_and_waveform(self, freq, waveform):
        """Set frequency and waveform"""
        freq = max(MIN_FREQ, min(MAX_FREQ, freq))
        
        # Calculate frequency word
        freq_word = int((freq * 268435456.0) / 25000000.0)
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
        
        # Update current settings
        self.current_frequency = freq
        self.current_waveform = waveform
        self.output_active = True
    
    def stop_output(self):
        """Stop waveform output"""
        self.write_ad9833(0x2040)  # Sleep mode
        self.output_active = False
    
    def format_frequency(self, frequency):
        """Format frequency for display"""
        if frequency >= 1000000:
            return f"{frequency/1000000:.3f} MHz"
        elif frequency >= 1000:
            return f"{frequency/1000:.2f} kHz"
        else:
            return f"{frequency:.1f} Hz"
    
    def get_waveform_info(self, waveform):
        """Get waveform amplitude info"""
        amplitudes = {
            "SINE": "~0.6Vpp",
            "TRIANGLE": "~0.6Vpp", 
            "SQUARE": "3.3Vpp",
            "SQUARE/2": "3.3Vpp"
        }
        return amplitudes.get(waveform, "Unknown")
    
    def show_main_menu(self):
        """Display main menu"""
        print("\n" + "=" * 50)
        print("*** AD9833 AWG CONTROL MENU ***")
        print("=" * 50)
        print(">> Current Status:")
        print(f"   Frequency: {self.format_frequency(self.current_frequency)}")
        print(f"   Waveform:  {self.current_waveform}")
        print(f"   Amplitude: {self.get_waveform_info(self.current_waveform)}")
        print(f"   Output:    {'[ON]' if self.output_active else '[OFF]'}")
        print("-" * 50)
        print(">> Menu Options:")
        print("   [1] Set Frequency")
        print("   [2] Set Waveform")
        print("   [3] Run Waveform")
        print("   [4] Stop Waveform")
        print("   [5] Manual Mode (DIP + Potentiometer)")
        print("   [6] Quick Presets")
        print("   [7] System Info")
        print("   [0] Exit")
        print("=" * 50)
    
    def frequency_selection_menu(self):
        """Frequency selection submenu"""
        print("\n*** FREQUENCY SELECTION ***")
        print("-" * 30)
        print("Current:", self.format_frequency(self.current_frequency))
        print("\nOptions:")
        print("[1] Enter exact frequency")
        print("[2] Quick frequency presets")
        print("[3] Back to main menu")
        
        try:
            choice = input("\nSelect option (1-3): ").strip()
            
            if choice == "1":
                freq_str = input("Enter frequency (Hz): ").strip()
                try:
                    new_freq = float(freq_str)
                    if MIN_FREQ <= new_freq <= MAX_FREQ:
                        self.current_frequency = new_freq
                        print(f"[OK] Frequency set to {self.format_frequency(new_freq)}")
                    else:
                        print(f"[ERROR] Frequency must be between {MIN_FREQ}Hz and {MAX_FREQ}Hz")
                except ValueError:
                    print("[ERROR] Invalid frequency format")
            
            elif choice == "2":
                self.quick_frequency_presets()
            
        except KeyboardInterrupt:
            print("\n<< Returning to main menu")
    
    def quick_frequency_presets(self):
        """Quick frequency preset selection"""
        presets = [
            (1, "1 Hz"),
            (10, "10 Hz"),
            (100, "100 Hz"),
            (1000, "1 kHz"),
            (10000, "10 kHz"),
            (100000, "100 kHz"),
            (440, "440 Hz (A4 note)"),
            (1000000, "1 MHz")
        ]
        
        print("\n*** QUICK FREQUENCY PRESETS ***")
        print("-" * 30)
        for i, (freq, name) in enumerate(presets, 1):
            print(f"[{i}] {name}")
        
        try:
            choice = input(f"\nSelect preset (1-{len(presets)}): ").strip()
            preset_idx = int(choice) - 1
            
            if 0 <= preset_idx < len(presets):
                freq, name = presets[preset_idx]
                self.current_frequency = freq
                print(f"[OK] Frequency set to {name}")
            else:
                print("[ERROR] Invalid preset selection")
                
        except (ValueError, KeyboardInterrupt):
            print("<< Returning to frequency menu")
    
    def waveform_selection_menu(self):
        """Waveform selection menu"""
        waveforms = ["SINE", "TRIANGLE", "SQUARE", "SQUARE/2"]
        
        print("\n*** WAVEFORM SELECTION ***")
        print("-" * 30)
        print("Current:", self.current_waveform)
        print("\nAvailable waveforms:")
        
        for i, waveform in enumerate(waveforms, 1):
            amplitude = self.get_waveform_info(waveform)
            marker = ">>>" if waveform == self.current_waveform else "   "
            print(f"{marker} [{i}] {waveform:<10} ({amplitude})")
        
        try:
            choice = input(f"\nSelect waveform (1-{len(waveforms)}): ").strip()
            waveform_idx = int(choice) - 1
            
            if 0 <= waveform_idx < len(waveforms):
                self.current_waveform = waveforms[waveform_idx]
                amplitude = self.get_waveform_info(self.current_waveform)
                print(f"[OK] Waveform set to {self.current_waveform} ({amplitude})")
            else:
                print("[ERROR] Invalid waveform selection")
                
        except (ValueError, KeyboardInterrupt):
            print("<< Returning to main menu")
    
    def run_waveform(self):
        """Start waveform output"""
        print(f"\n*** STARTING WAVEFORM OUTPUT ***")
        print("-" * 35)
        print(f"Frequency: {self.format_frequency(self.current_frequency)}")
        print(f"Waveform:  {self.current_waveform}")
        print(f"Amplitude: {self.get_waveform_info(self.current_waveform)}")
        
        try:
            self.set_frequency_and_waveform(self.current_frequency, self.current_waveform)
            print("[OK] Waveform output STARTED")
            print(">> Check your oscilloscope")
            
        except Exception as e:
            print(f"[ERROR] Error starting waveform: {e}")
    
    def stop_waveform_menu(self):
        """Stop waveform output"""
        if self.output_active:
            print("\n*** STOPPING WAVEFORM OUTPUT ***")
            print("-" * 30)
            self.stop_output()
            print("[OK] Waveform output STOPPED")
        else:
            print("\n[WARNING] No waveform currently active")
    
    def manual_mode(self):
        """Manual mode with DIP switches and potentiometer"""
        print("\n*** MANUAL MODE ***")
        print("=" * 40)
        print(">> Use potentiometer to control frequency")
        print(">> Use DIP switches to select waveform:")
        print("   SW1=OFF SW2=OFF -> SINE")
        print("   SW1=OFF SW2=ON  -> TRIANGLE")
        print("   SW1=ON  SW2=OFF -> SQUARE")
        print("   SW1=ON  SW2=ON  -> SQUARE/2")
        print("\n>> Press Ctrl+C to return to menu")
        print("=" * 40)
        
        # Reset state tracking
        self.prev_pot_value = None
        self.prev_sw1 = None
        self.prev_sw2 = None
        
        try:
            while True:
                # Read inputs
                pot_raw = self.freq_pot.read_u16()
                pot_value = pot_raw >> 4
                frequency = self.map_logarithmic(pot_value, 0, 4095, MIN_FREQ, MAX_FREQ)
                
                sw1_state = not self.sw1.value()
                sw2_state = not self.sw2.value()
                
                # Determine waveform
                if not sw1_state and not sw2_state:
                    waveform = "SINE"
                elif not sw1_state and sw2_state:
                    waveform = "TRIANGLE"
                elif sw1_state and not sw2_state:
                    waveform = "SQUARE"
                else:
                    waveform = "SQUARE/2"
                
                # Check for changes
                pot_changed = (self.prev_pot_value is None or 
                             abs(pot_value - self.prev_pot_value) > 20)
                switch_changed = (self.prev_sw1 != sw1_state or 
                                self.prev_sw2 != sw2_state)
                
                # Update if changed
                if pot_changed or switch_changed:
                    self.set_frequency_and_waveform(frequency, waveform)
                    
                    print(f">> {self.format_frequency(frequency)} | {waveform} | "
                          f"SW1={'ON' if sw1_state else 'OFF'} SW2={'ON' if sw2_state else 'OFF'}")
                    
                    self.prev_pot_value = pot_value
                    self.prev_sw1 = sw1_state
                    self.prev_sw2 = sw2_state
                
                utime.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n<< Exiting manual mode")
    
    def map_logarithmic(self, input_val, in_min, in_max, out_min, out_max):
        """Logarithmic mapping for frequency"""
        normalized = (input_val - in_min) / (in_max - in_min)
        log_min = math.log10(out_min)
        log_max = math.log10(out_max)
        log_value = log_min + normalized * (log_max - log_min)
        return 10 ** log_value
    
    def quick_presets_menu(self):
        """Quick preset configurations"""
        presets = [
            (440, "SINE", "♪ A4 Musical Note"),
            (1000, "SQUARE", "Clock 1kHz Clock Signal"),
            (10000, "TRIANGLE", "/\\ 10kHz Triangle"),
            (100000, "SINE", "RF 100kHz RF Test"),
            (1, "SQUARE", "Slow 1Hz Slow Pulse")
        ]
        
        print("\n*** QUICK PRESETS ***")
        print("-" * 30)
        for i, (freq, wave, desc) in enumerate(presets, 1):
            print(f"[{i}] {desc}")
            print(f"    {self.format_frequency(freq)} {wave}")
        
        try:
            choice = input(f"\nSelect preset (1-{len(presets)}): ").strip()
            preset_idx = int(choice) - 1
            
            if 0 <= preset_idx < len(presets):
                freq, wave, desc = presets[preset_idx]
                self.current_frequency = freq
                self.current_waveform = wave
                self.set_frequency_and_waveform(freq, wave)
                print(f"[OK] Preset loaded: {desc}")
            else:
                print("[ERROR] Invalid preset selection")
                
        except (ValueError, KeyboardInterrupt):
            print("<< Returning to main menu")
    
    def system_info(self):
        """Display system information"""
        print("\n*** SYSTEM INFORMATION ***")
        print("=" * 40)
        print(">> Hardware:")
        print("   Microcontroller: Raspberry Pi Pico")
        print("   AWG Chip: AD9833 DDS")
        print("   SPI Frequency: 1 MHz")
        print("   Crystal: 25 MHz")
        print()
        print(">> Specifications:")
        print(f"   Frequency Range: {MIN_FREQ} Hz - {self.format_frequency(MAX_FREQ)}")
        print("   Waveforms: SINE, TRIANGLE, SQUARE")
        print("   Sine/Triangle: ~0.6V peak-to-peak")
        print("   Square waves: 3.3V peak-to-peak")
        print()
        print(">> Controls:")
        print("   Potentiometer: GPIO26 (ADC0)")
        print("   DIP Switch 1: GPIO20")
        print("   DIP Switch 2: GPIO21")
        print("   AD9833 FSYNC: GPIO17")
        print()
        print(">> ASCII Waveform Examples:")
        print("   SINE:     ~~~^~~~v~~~^~~~v~~~")
        print("   TRIANGLE: /\\/\\/\\/\\/\\/\\/\\/\\")
        print("   SQUARE:   |‾|_|‾|_|‾|_|‾|_|‾|")
        print("=" * 40)
        
        input("\nPress Enter to continue...")
    
    def run_menu_system(self):
        """Main menu loop"""
        print(">> AWG System Started!")
        
        while True:
            try:
                self.show_main_menu()
                choice = input("\n>> Select option (0-7): ").strip()
                
                if choice == "1":
                    self.frequency_selection_menu()
                elif choice == "2":
                    self.waveform_selection_menu()
                elif choice == "3":
                    self.run_waveform()
                elif choice == "4":
                    self.stop_waveform_menu()
                elif choice == "5":
                    self.manual_mode()
                elif choice == "6":
                    self.quick_presets_menu()
                elif choice == "7":
                    self.system_info()
                elif choice == "0":
                    print("\n<< Goodbye! AWG system shutting down...")
                    self.stop_output()
                    break
                else:
                    print("[ERROR] Invalid option. Please select 0-7.")
                    
            except KeyboardInterrupt:
                print("\n\n[WARNING] Interrupted! Returning to menu...")
                continue
            except Exception as e:
                print(f"\n[ERROR] Error: {e}")
                print("Returning to main menu...")

# Main execution
if __name__ == "__main__":
    try:
        awg = AD9833_MenuDriven()
        awg.run_menu_system()
        
    except KeyboardInterrupt:
        print("\n\n[STOP] System interrupted by user")
    except Exception as e:
        print(f"\n[FATAL] System error: {e}")
    finally:
        print("[EXIT] AWG system terminated")
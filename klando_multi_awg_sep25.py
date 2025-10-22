from machine import Pin, ADC, I2C, SPI 
import time
from ssd1306 import SSD1306_I2C
from ina219 import INA219

# --- Hardware Setup ---
# I2C for OLED and INA219 (GP0: SDA, GP1: SCL)
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)
# Initialize INA219. The driver (ina219.py) must be in the same folder.
ina219 = INA219(i2c) 

# ADC for Resistance (GP26 / ADC0)
adc = ADC(26)

# Push Button (GP16)
button = Pin(16, Pin.IN, Pin.PULL_UP)
button_state = 1
last_button_press = 0

# Buzzer (GP17)
buzzer = Pin(17, Pin.OUT, Pin.PULL_DOWN)

# Resistance Circuit (R_REF and Bleed Resistor)
VCC_3V3 = 3.3
R_REF = 55000  # 55 kOhm
R_BLEED = 1000000 # 1 MOhm

# AD9833 Waveform Generator (***HARDWARE SPI CORREGIDO***)
# Nota: SPI(0) requiere SCK en GP2, 6, 18, 22. Usaremos GP2/GP3/GP4 por conveniencia.
# Conexiones actualizadas:
# GP4: FNC (Chip Select) - Flexible, no de hardware SPI
# GP2: CLK (SCK for SPI) - Requerido para SPI(0)
# GP3: DAT (MOSI for SPI) - Requerido para SPI(0)
AD9833_FNC_PIN = Pin(4, Pin.OUT) # Cambiado de GP8 a GP4
# Inicialización de SPI(0) en los pines correspondientes al AD9833
spi = SPI(0, baudrate=1000000, polarity=0, phase=1, sck=Pin(2), mosi=Pin(3)) # CORRECCIÓN: SCK=GP2, MOSI=GP3

# --- Global State ---
MEASUREMENT_MODES = [
    ("Bus Voltage", "V", 2),
    ("Current", "mA", 2),
    ("Power", "mW", 2),
    ("Resistance", "Ohm", 1),
    ("Continuity", "", 0),
    ("Waveform Gen", "Hz", 0)
]
current_display_mode_index = 0

# --- AD9833 Class ---
class AD9833:
    # Ahora recibe el objeto SPI y el pin FNC
    def __init__(self, spi, fnc_pin):
        self.spi = spi
        self.fnc = fnc_pin
        self.fnc.value(1) # Start with FSYNC high
        
        self.waveforms = {
            "Sine": 0x2000,
            "Triangle": 0x2002,
            "Square": 0x2028
        }
        self.frequencies = [100, 1000, 5000, 10000, 50000, 100000] # Frequencies in Hz
        self.current_waveform = "Sine"
        self.current_freq_index = 0
        self.initialize()
        
    def write_word(self, word):
        # *** Implementación con Hardware SPI ***
        # Usa spi.write para enviar el word de 16 bits como 2 bytes (MSB primero)
        data = word.to_bytes(2, 'big') 
        
        self.fnc.value(0) # FSYNC bajo para iniciar la transmisión
        self.spi.write(data)
        self.fnc.value(1) # FSYNC alto para finalizar la transmisión
        time.sleep_us(5) # CORRECCIÓN: Pequeño retardo aumentado para robustez del timing.
        
    def initialize(self):
        self.fnc.value(0)
        self.fnc.value(1) # Pulso de Reset
        self.write_word(0x0100) # Reset de Software
        self.write_word(0x2100) # Control word (set default mode)
        self.write_word(0x4000)
        self.write_word(0x5000)
        self.write_word(0x2000) # Sale del reset, establece el control predeterminado
        self.set_frequency()
        self.set_waveform()

    def set_frequency(self, freq=None):
        if freq is None:
            freq = self.frequencies[self.current_freq_index]
        
        # Cálculo de la palabra de frecuencia: F_WORD = (F_OUT * 2^28) / F_MCLK
        freq_hex = int(freq * 10.73741824) 
        lsb = freq_hex & 0x3FFF
        msb = (freq_hex >> 14) & 0x3FFF
        
        # Escribe en el Registro de Frecuencia 0 (FRQ0)
        self.write_word(0x2100) # Control Register: B28=1, FSEL=0 (escribir en FRQ0)
        self.write_word(0x4000 | lsb)
        self.write_word(0x5000 | msb)
        
        # Escribe el Registro de Control (Selecciona FRQ0 y la forma de onda actual)
        self.write_word(0x2000 | self.waveforms[self.current_waveform])

    def set_waveform(self, waveform=None):
        if waveform is not None:
            self.current_waveform = waveform
        
        # Escribe el Registro de Control, aplicando la forma de onda seleccionada
        self.write_word(self.waveforms[self.current_waveform])

    def next_freq(self):
        self.current_freq_index = (self.current_freq_index + 1) % len(self.frequencies)
        self.set_frequency()

    def next_waveform(self):
        waveforms_list = list(self.waveforms.keys())
        current_index = waveforms_list.index(self.current_waveform)
        self.current_waveform = waveforms_list[(current_index + 1) % len(waveforms_list)]
        self.set_waveform()

# --- Main Functions ---
def display_text(label, value_str, unit, decimal_places):
    oled.fill(0)
    oled.text(label, 0, 0)
    
    # Large font for value
    display_value_str = "{:.{prec}f}".format(value_str, prec=decimal_places)
    oled.text(display_value_str, 0, 20)

    # Unit in smaller font
    oled.text(unit, 128 - (len(unit) * 8), 20)
    oled.show()

def get_resistance_value():
    adc_value = adc.read_u16()
    adc_voltage = adc_value * VCC_3V3 / 65535
    
    # Voltage divider formula R_TEST = R_REF * (V_ADC / (V_REF - V_ADC))
    if VCC_3V3 - adc_voltage == 0:
        return float('inf')
    
    try:
        resistance = R_REF * (adc_voltage / (VCC_3V3 - adc_voltage))
    except ZeroDivisionError:
        resistance = float('inf')
    
    return resistance

def get_resistance_string(resistance):
    if resistance > 500000:
        return "Open"
    elif resistance >= 1000000:
        return f"{resistance / 1000000:.2f}MΩ"
    elif resistance >= 1000:
        return f"{resistance / 1000:.2f}kΩ"
    else:
        return f"{resistance:.1f}Ω"

def run_awg_mode():
    oled.fill(0)
    oled.text("AWG - Generador", 0, 0)
    oled.text("Tipo:", 0, 16)
    oled.text(ad9833.current_waveform, 40, 16)
    oled.text("Freq:", 0, 32)
    oled.text(str(ad9833.frequencies[ad9833.current_freq_index]), 40, 32)
    oled.text("Hz", 96, 32)
    oled.text("Short=Freq, Long=Wave", 0, 52)
    oled.show()

# Initialize the AD9833 using the hardware SPI object
ad9833 = AD9833(spi, AD9833_FNC_PIN)

# --- Main Loop ---
print("Iniciando Multímetro/AWG...")
while True:
    current_time_ms = time.ticks_ms()
    
    # Check for button press
    if button.value() == 0 and button_state == 1:
        button_state = 0
        last_button_press = current_time_ms

    if button.value() == 1 and button_state == 0:
        button_state = 1
        press_duration = time.ticks_diff(current_time_ms, last_button_press)
        
        if MEASUREMENT_MODES[current_display_mode_index][0] == "Waveform Gen":
            # Waveform Gen mode logic
            if press_duration < 1000:  # Short press for frequency change
                ad9833.next_freq()
            else:  # Long press for waveform type change
                ad9833.next_waveform()
        else:
            # Multimeter mode logic
            current_display_mode_index = (current_display_mode_index + 1) % len(MEASUREMENT_MODES)

    mode_label, mode_unit, mode_decimal_places = MEASUREMENT_MODES[current_display_mode_index]

    oled.fill(0)
    
    if mode_label == "Bus Voltage":
        bus_voltage = ina219.get_bus_voltage()
        display_text(mode_label, bus_voltage, mode_unit, mode_decimal_places)
        
    elif mode_label == "Current":
        current = ina219.get_current() * 1000 # Convert to mA
        display_text(mode_label, current, mode_unit, mode_decimal_places)
        
    elif mode_label == "Power":
        power = ina219.get_power() * 1000 # Convert to mW
        display_text(mode_label, power, mode_unit, mode_decimal_places)
        
    elif mode_label == "Resistance":
        # Average 10 samples for stable reading
        resistance_sum = 0
        for _ in range(10):
            resistance_sum += get_resistance_value()
            time.sleep_ms(1)
        resistance = resistance_sum / 10

        if resistance == float('inf'):
            oled.fill(0)
            oled.text("Resistencia", 0, 0)
            oled.text("Circuito Abierto", 0, 32)
            oled.show()
        else:
            resistance_str = get_resistance_string(resistance)
            oled.fill(0)
            oled.text("Resistencia", 0, 0)
            oled.text(resistance_str, 0, 32)
            oled.show()
            
    elif mode_label == "Continuity":
        resistance = get_resistance_value()
        oled.fill(0)
        oled.text("Continuidad", 0, 0)
        
        if resistance < 3: # 3 Ohm threshold
            oled.text("BEEP! Conectado", 0, 32)
            buzzer.value(1)
            time.sleep_ms(200)
            buzzer.value(0)
        else:
            oled.text("Abierto", 0, 32)
        oled.show()

    elif mode_label == "Waveform Gen":
        run_awg_mode()

    # Small delay for responsiveness and stability
    time.sleep_ms(10)

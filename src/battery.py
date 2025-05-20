# MicroPython Battery Configuration
#
# The MIT License (MIT)
#
# Copyright (c) Shariar Imam Shafin
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from machine import ADC, Pin
import time

# ADC setup for GPIO34
adc = ADC(Pin(34))
adc.atten(ADC.ATTN_11DB)   # Full range approx 0-3.3V
adc.width(ADC.WIDTH_12BIT) # 12-bit ADC (0-4095)

# Voltage divider resistor values (ohms)
R1 = 99100.0  # (+/- 100kΩ) My one is 99.1kΩ
R2 = 99100.0  # (+/- 100kΩ) My one is 99.1kΩ

# Calibration factor to adjust ADC voltage reading
calibration_factor = 1.14
voltage_offset = 0.0

def read_battery_voltage(state=None):
    # Initialize state if not provided
    if state is None:
        state = {'samples': 0, 'total': 0, 'last_sample_time': time.ticks_ms()}
    
    now = time.ticks_ms()
    # Check if 10ms (SAMPLE_INTERVAL) has passed since last sample
    SAMPLE_INTERVAL = 10  # 10ms between samples
    if time.ticks_diff(now, state['last_sample_time']) >= SAMPLE_INTERVAL:
        state['total'] += adc.read()
        state['samples'] += 1
        state['last_sample_time'] = now
        
        # If 20 samples are collected, calculate and return voltage
        if state['samples'] >= 20:
            raw = state['total'] / 20
            voltage_at_pin = (raw / 4095) * 3.3
            battery_voltage = voltage_at_pin * ((R1 + R2) / R2) * calibration_factor + voltage_offset
            return raw, voltage_at_pin, battery_voltage, None  # None indicates completion
        else:
            return None, None, None, state  # Return state to continue sampling
    return None, None, None, state  # Not time to sample yet

def voltage_to_percent(v):
    # Piecewise linear approximation for 2000mAh LiPo/18650 discharge curve
    if v >= 4.2:
        return 100
    elif v <= 3.2:
        return 0
    elif v > 4.0:  # 4.0V–4.2V: 80%–100%
        return int(80 + (v - 4.0) / (4.2 - 4.0) * 20)
    elif v > 3.8:  # 3.8V–4.0V: 50%–80%
        return int(50 + (v - 3.8) / (4.0 - 3.8) * 30)
    elif v > 3.5:  # 3.5V–3.8V: 20%–50%
        return int(20 + (v - 3.5) / (3.8 - 3.5) * 30)
    else:  # 3.2V–3.5V: 0%–20%
        return int((v - 3.2) / (3.5 - 3.2) * 20)

# MicroPython Weather Station Mini ESP32
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
#
from machine import Pin, I2C, SPI, deepsleep
import sh1106
import network
import urequests
import time
import ntptime
import ujson
from umodbus.serial import Serial as ModbusRTUMaster
import config
import battery

# Disable Bluetooth to save power
try:
    import machine
    print("Bluetooth disabled (not initialized)")
except Exception as e:
    print("Error disabling Bluetooth:", e)

# I2C setup for BME280
i2c = I2C(1, scl=Pin(19), sda=Pin(21), freq=400000)

# SPI setup for SH1106
spi = SPI(2, baudrate=2000000, sck=Pin(18), mosi=Pin(23))
dc = Pin(12)
rst = Pin(13)
cs = Pin(5)
oled = sh1106.SH1106_SPI(128, 64, spi, dc, rst, cs)
oled.flip(True, True)
oled.contrast(255)

# Button setup
button = Pin(14, Pin.IN, Pin.PULL_UP)

# Modbus setup
rtu_pins = (Pin(16), Pin(17))
host = ModbusRTUMaster(
    baudrate=4800,
    data_bits=8,
    stop_bits=1,
    parity=None,
    pins=rtu_pins,
    ctrl_pin=None,
    uart_id=2
)
print("Modbus UART initialized, baudrate=4800, pins=(16, 17)")

# Timing constants (ms)
MAIN_LOOP_INTERVAL = 120000     # 120s for battery savings
DISPLAY_REFRESH = 1000          # 1s for display
WEATHER_UPDATE_INTERVAL = 300000 # 5min for weather
THINGSPEAK_INTERVAL = 300000    # 5min for ThingSpeak
TIME_SYNC_INTERVAL = 3600000    # 1hr for NTP
DEBOUNCE_DELAY = 30             # 30ms for button debounce
LONG_PRESS_DURATION = 1000      # 1s for long press
WIFI_CHECK_INTERVAL = 500       # 500ms Wi-Fi check
WIFI_RETRY_COUNT = 3            # Retry Wi-Fi 3 times
NTP_RETRY_COUNT = 3             # Retry NTP 3 times
WEATHER_RETRY_COUNT = 3         # Retry weather 3 times
NTP_RETRY_INTERVAL = 1000       # 1s for NTP retry
BME_READ_INTERVAL = 200         # 200ms for BME280 read
BME_WARMUP_COUNT = 5            # 5 dummy reads for warm-up
WIFI_TIMEOUT = 120000           # 2min for Wi-Fi failure
OLED_TIMEOUT = 300000           # 5min for OLED off
WIFI_RETRY_INTERVAL = 120000    # 2min between Wi-Fi retries
BLINK_INTERVAL = 500            # 500ms for warning blink
MODBUS_RETRY_COUNT = 3          # Retry Modbus 3 times
MODBUS_RETRY_INTERVAL = 1000    # 1s between Modbus retries

# Global state
current_page = 1
last_button_time = 0
is_button_pressed = False
button_press_start = 0
last_button_press = 0
weather_temp = None
weather_hum = None
weather_condition = "N/A"
last_weather_update = 0
temp = None
hum = None
pressure = None
altitude = None
wifi_ok = False
sensor_ok = False
bme = None
wind_speed = None
sensor_status = None
modbus_error = None
last_display_update = 0
last_thingspeak_update = 0
display_buffer = {}
last_weather_code = None
oled_on = True
wifi_start_time = 0
battery_percent = None
next_warning_time = 0
ignore_first_press = False
is_warning_active = False  # Flag to prevent OLED timeout during warning
blink_state = 0           # For non-blocking warning blink
blink_count = 0
blink_start_time = 0
battery_state = None      # For non-blocking battery voltage reading
ntp_attempt = None        # NTP globals
ntp_start_time = None
ntp_retry_start = None
weather_attempt = None    # Weather globals
weather_start_time = None
weather_retry_start = None

# Open-Meteo weather codes
WEATHER_CODES = {
    0: "Clear", 1: "Clear", 2: "Cloudy", 3: "Cloudy",
    10: "Mist", 11: "Fog", 12: "Fog",
    20: "Drizzle", 21: "Drizzle", 22: "Drizzle",
    30: "Rain", 31: "Rain", 32: "Rain",
    40: "Rain", 41: "Rain",
    50: "Rain", 51: "Rain",
    60: "Rain", 61: "Rain", 62: "Rain",
    70: "Rain", 71: "Rain", 72: "Rain",
    80: "Rain", 81: "Rain", 82: "Rain",
    90: "Storm", 91: "Storm", 92: "Storm"
}

def connect_wifi():
    global wifi_ok, wifi_start_time
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wifi_start_time = time.ticks_ms()
    attempt = 0
    wifi_retry_start = 0
    connection_start = time.ticks_ms()
    while attempt < WIFI_RETRY_COUNT:
        now = time.ticks_ms()
        if wifi_retry_start and time.ticks_diff(now, wifi_retry_start) < WIFI_RETRY_INTERVAL:
            continue  # Wait for 2-minute retry interval
        print("WiFi attempt", attempt + 1)
        wlan.connect(config.SSID, config.PASSWORD)
        connection_start = now
        while time.ticks_diff(now, connection_start) < 5000:
            if wlan.isconnected():
                print("Connected to WiFi, IP:", wlan.ifconfig()[0])
                wifi_ok = True
                return True
            now = time.ticks_ms()
        attempt += 1
        if attempt < WIFI_RETRY_COUNT:
            print(f"Wi-Fi failed, retrying in 2 minutes (attempt {attempt}/{WIFI_RETRY_COUNT})")
            wifi_retry_start = time.ticks_ms()
    print("WiFi connection failed after 3 attempts")
    wifi_ok = False
    return False

def enter_deep_sleep():
    print("Entering deep sleep, wake on button press (GPIO 14)")
    oled.poweroff()
    button = Pin(14, Pin.IN, Pin.PULL_UP)
    machine.deepsleep()

def sync_time():
    global ntp_attempt, ntp_start_time, ntp_retry_start
    now = time.ticks_ms()
    if ntp_attempt is None:
        ntp_attempt = 0
        ntp_start_time = now
        ntp_retry_start = 0
    if ntp_retry_start and time.ticks_diff(now, ntp_retry_start) < NTP_RETRY_INTERVAL:
        return False
    if ntp_attempt >= NTP_RETRY_COUNT:
        print("Time sync failed")
        ntp_attempt = 0
        return False
    print("NTP attempt", ntp_attempt + 1)
    try:
        ntptime.settime()
        print("Time synchronized")
        ntp_attempt = 0
        return True
    except Exception as e:
        print("NTP failed:", e)
        ntp_attempt += 1
        ntp_retry_start = now
        return False

def update_weather():
    global weather_temp, weather_hum, weather_condition, last_weather_update, last_weather_code, weather_attempt, weather_start_time, weather_retry_start
    now = time.ticks_ms()
    if weather_attempt is None:
        weather_attempt = 0
        weather_start_time = now
        weather_retry_start = 0
    if weather_retry_start and time.ticks_diff(now, weather_retry_start) < 1000:
        return
    if weather_attempt >= WEATHER_RETRY_COUNT:
        print("Weather API unavailable")
        weather_attempt = 0
        return
    print("Weather API attempt", weather_attempt + 1)
    try:
        response = urequests.get(config.WEATHER_URL, headers={'User-Agent': 'MicroPython/1.0'})
        print("Weather API status:", response.status_code)
        if response.status_code == 200:
            data = ujson.loads(response.text)
            weather_temp = data["current"]["temperature_2m"]
            weather_hum = data["current"]["relative_humidity_2m"]
            last_weather_code = data["current"]["weather_code"]
            weather_condition = WEATHER_CODES.get(last_weather_code, "Unknown")
            last_weather_update = now
            print(f"Weather updated: Temp={weather_temp}C, Hum={weather_hum}%, Code={last_weather_code}, Condition={weather_condition}")
            response.close()
            weather_attempt = 0
            return
        else:
            print("Weather API failed: Status", response.status_code)
            weather_condition = "HTTP Error"
            response.close()
    except Exception as e:
        print("Weather API failed:", e)
        weather_condition = "API Error"
    weather_attempt += 1
    weather_retry_start = now

def get_sensor_data():
    try:
        if bme is None:
            return None, None, None, None
        t, p, h = bme.values()
        a = 44330 * (1 - (p / 1013.25) ** (1/5.255))
        print(f"BME280: Temp={t:.1f}C, Hum={h:.1f}%, Press={p:.0f}hPa, Alt={a:.1f}m")
        return t, h, p, a
    except Exception as e:
        print("BME280 error:", e)
        return None, None, None, None

def read_modbus():
    global wind_speed, sensor_status, modbus_error
    attempt = 0
    while attempt < MODBUS_RETRY_COUNT:
        try:
            print(f"Reading Modbus registers (slave_addr=0x01, start_addr=0x00, qty=2, attempt {attempt + 1}/{MODBUS_RETRY_COUNT})")
            values = host.read_holding_registers(slave_addr=0x01, starting_addr=0x00, register_qty=2, signed=False)
            wind_speed = (values[0] / 10.0) * 3.6
            sensor_status = values[1]
            modbus_error = None
            print(f"Modbus: Wind Speed={wind_speed:.1f} km/h, Status={sensor_status}")
            return
        except Exception as e:
            print(f"Modbus error (attempt {attempt + 1}):", str(e))
            modbus_error = str(e)
            attempt += 1
            if attempt < MODBUS_RETRY_COUNT:
                time.sleep_ms(MODBUS_RETRY_INTERVAL)
    wind_speed = None
    sensor_status = None
    print(f"Modbus failed after {MODBUS_RETRY_COUNT} attempts")

def upload_to_thingspeak(temp, hum, pressure, wind_speed):
    if temp is None or hum is None or pressure is None:
        print("ThingSpeak: Incomplete data, skipping upload (temp={}, hum={}, pressure={}, wind_speed={})".format(temp, hum, pressure, wind_speed))
        return
    try:
        url = f"http://api.thingspeak.com/update?api_key={config.THINGSPEAK_API_KEY}&field1={temp}&field2={hum}&field3={pressure}&field4={wind_speed}"
        print("ThingSpeak URL:", url.replace(config.THINGSPEAK_API_KEY, "XXX"))
        response = urequests.get(url)
        print("ThingSpeak status code:", response.status_code)
        if response.status_code == 200:
            print("ThingSpeak response:", response.text)
        else:
            print("ThingSpeak failed with status:", response.status_code)
        response.close()
    except Exception as e:
        print("ThingSpeak upload failed:", str(e))

def manage_oled():
    global oled_on
    now = time.ticks_ms()
    if oled_on and not is_warning_active and time.ticks_diff(now, last_button_press) >= OLED_TIMEOUT:
        print("OLED timeout, turning off")
        oled.fill(0)
        oled.show()
        oled.poweroff()
        oled_on = False

def restore_oled():
    global oled_on, ignore_first_press
    if not oled_on:
        print("Restoring OLED")
        oled.poweron()
        oled.fill(0)
        oled.contrast(255)
        oled_on = True
        ignore_first_press = True
        display_data()

def blink_warning():
    global is_warning_active, last_button_press, blink_state, blink_count, blink_start_time
    now = time.ticks_ms()
    if not is_warning_active:
        is_warning_active = True
        blink_state = 0
        blink_count = 0
        blink_start_time = now
        oled.fill(0)
    if time.ticks_diff(now, blink_start_time) >= BLINK_INTERVAL:
        if blink_state == 0:
            oled.fill(0)
            oled.text("Low Battery", 20, 30)
            oled.show()
            blink_state = 1
        else:
            oled.fill(0)
            oled.show()
            blink_state = 0
            blink_count += 1
        blink_start_time = now
    if blink_count >= 3:
        is_warning_active = False
        blink_count = 0
        display_data()
        last_button_press = now
        return True
    return False

def display_data():
    global last_display_update, display_buffer
    if not oled_on:
        return
    now = time.ticks_ms()
    if time.ticks_diff(now, last_display_update) < DISPLAY_REFRESH:
        return

    current_time = time.localtime(time.time() + 6 * 3600)
    new_buffer = {}

    if current_page == 1:
        hr = current_time[3] % 12 or 12
        time_str = f"{hr:02d}:{current_time[4]:02d} {'AM' if current_time[3] < 12 else 'PM'}"
        bat_str = f"{battery_percent}%" if battery_percent is not None else "N/A"
        new_buffer['header_left'] = time_str
        new_buffer['header_right'] = bat_str
        new_buffer['lines'] = []
        if not wifi_ok:
            new_buffer['lines'].append("WiFi Error")
        elif not sensor_ok:
            new_buffer['lines'].append("Sensor Error")
        elif temp is None:
            new_buffer['lines'].append("Sensor Fail")
        else:
            new_buffer['lines'] = [
                f"Temp {temp:>7.1f} C",
                f"Humid{hum:>7.1f} %",
                f"Bar  {pressure:>7.0f} hPa",
                f"Alt  {altitude:>7.1f} m"
            ]
    elif current_page == 2:
        day = ["Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"][current_time[6]]
        date_str = f"{current_time[2]}/{current_time[1]}/{current_time[0] % 100:02d}"
        new_buffer['header_left'] = day
        new_buffer['header_right'] = date_str
        new_buffer['lines'] = ["Dhaka Weather"]
        if weather_temp is not None:
            new_buffer['lines'].extend([
                f"Temp  {weather_temp:>7.1f} C",
                f"Humid {weather_hum:>7.1f} %",
                weather_condition
            ])
        else:
            new_buffer['lines'].append("No Weather Data")
    else:
        new_buffer['header_left'] = "Modbus"
        new_buffer['header_right'] = "0x00"
        new_buffer['lines'] = []
        if modbus_error:
            new_buffer['lines'].extend(["Modbus Error", modbus_error[:15]])
        elif wind_speed is None:
            new_buffer['lines'].append("No Wind Data")
        else:
            new_buffer['lines'].extend([
                f"Speed {wind_speed:>5.1f} km/h",
                f"Status {sensor_status:>4d}"
            ])

    if new_buffer != display_buffer:
        oled.fill(0)
        oled.text(new_buffer['header_left'], 0, 0)
        oled.text(new_buffer['header_right'], 128 - len(new_buffer['header_right']) * 8, 0)
        oled.text("----------------------", 0, 9)
        for i, line in enumerate(new_buffer['lines']):
            oled.text(line, 0, 18 + i * 10)
        oled.show()
        display_buffer = new_buffer
    last_display_update = now

def recalibrate():
    print("Recalibrating...")
    oled.fill(0)
    oled.text("Calibrating...", 20, 30)
    oled.show()
    if wifi_ok:
        sync_time()
        update_weather()
    if sensor_ok:
        global temp, hum, pressure, altitude
        temp, hum, pressure, altitude = get_sensor_data()
    if current_page == 3:
        read_modbus()
    display_data()
    print("Recalibration complete")

def button_handler(pin):
    global current_page, last_button_time, is_button_pressed, button_press_start, last_button_press, ignore_first_press
    now = time.ticks_ms()
    if time.ticks_diff(now, last_button_time) < DEBOUNCE_DELAY:
        return

    if pin.value() == 0 and not is_button_pressed:
        is_button_pressed = True
        button_press_start = now
        last_button_press = now
        print("Button pressed")
        restore_oled()
    elif pin.value() == 1 and is_button_pressed:
        is_button_pressed = False
        last_button_time = now
        last_button_press = now
        if ignore_first_press:
            ignore_first_press = False
            print("Ignoring first button press after wake")
            return
        duration = time.ticks_diff(now, button_press_start)
        if duration >= LONG_PRESS_DURATION:
            print("Long press: recalibrating")
            recalibrate()
        else:
            print("Short press: switching to page", current_page % 3 + 1)
            current_page = (current_page % 3) + 1
            if current_page == 3:
                read_modbus()
            display_data()

def main():
    global bme, sensor_ok, temp, hum, pressure, altitude, wind_speed, sensor_status, modbus_error, battery_percent, wifi_ok, last_button_press, battery_state
    oled.fill(0)
    oled.text("Initializing...", 6, 30)
    oled.show()

    print("Initializing BME280")
    try:
        import bme280
        bme = bme280.BME280(i2c=i2c, address=0x76)
        print("BME280 warm-up started")
        warmup_count = 0
        warmup_start = time.ticks_ms()
        warmup_read_start = warmup_start
        while warmup_count < BME_WARMUP_COUNT:
            now = time.ticks_ms()
            if time.ticks_diff(now, warmup_read_start) >= BME_READ_INTERVAL:
                get_sensor_data()
                print(f"BME280 warm-up read {warmup_count + 1}/{BME_WARMUP_COUNT}")
                warmup_count += 1
                warmup_read_start = now
        stable_read_count = 0
        stable_read_start = time.ticks_ms()
        while stable_read_count < 3:
            now = time.ticks_ms()
            if time.ticks_diff(now, stable_read_start) >= BME_READ_INTERVAL:
                temp, hum, pressure, altitude = get_sensor_data()
                if temp is None:
                    print("BME280 read failed, retrying...")
                    stable_read_count += 1
                    stable_read_start = now
                else:
                    sensor_ok = True
                    print("BME280 initialized with stable reading")
                    break
        if not sensor_ok:
            print("BME280 failed to initialize")
    except Exception as e:
        print("BME280 initialization failed:", e)
        sensor_ok = False

    # Initialize Modbus data
    print("Reading initial Modbus data")
    read_modbus()

    # Initialize battery percentage (start non-blocking read)
    print("Initializing battery voltage reading")
    battery_state = None
    while True:
        raw, adc_v, batt_v, battery_state = battery.read_battery_voltage(battery_state)
        if battery_state is None:  # Sampling complete
            try:
                battery_percent = battery.voltage_to_percent(batt_v)
                print(f"Initial Battery Voltage: {batt_v:.3f}V, Percent: {battery_percent}%")
            except AttributeError as e:
                print("Battery module error:", e)
                battery_percent = 0
            break

    # Setup button interrupt
    print("Setting up button interrupt")
    try:
        button.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=button_handler)
    except Exception as e:
        print("Button interrupt setup failed:", e)

    # Attempt to connect to Wi-Fi with retries
    print("Attempting to connect to Wi-Fi")
    wifi_connected = connect_wifi()
    if wifi_connected:
        print("Wi-Fi connected, syncing time")
        sync_time()
        print("Fetching initial weather data")
        update_weather()
    else:
        print("Wi-Fi connection failed after 3 attempts. Operating in offline mode.")

    last_main_update = time.ticks_ms()
    last_time_sync = time.ticks_ms() if wifi_connected else 0
    last_thingspeak_update = time.ticks_ms() if wifi_connected else 0
    last_button_press = time.ticks_ms()
    battery_state = None  # Reset for main loop readings

    while True:
        now = time.ticks_ms()

        if not wifi_ok and time.ticks_diff(now, wifi_start_time) >= WIFI_TIMEOUT:
            enter_deep_sleep()

        manage_oled()

        if is_warning_active:
            blink_warning()

        if time.ticks_diff(now, last_main_update) >= MAIN_LOOP_INTERVAL:
            if sensor_ok:
                temp, hum, pressure, altitude = get_sensor_data()
            # Non-blocking battery voltage reading
            raw, adc_v, batt_v, battery_state = battery.read_battery_voltage(battery_state)
            if battery_state is None:  # Sampling complete
                try:
                    battery_percent = battery.voltage_to_percent(batt_v)
                    print(f"Battery Voltage: {batt_v:.3f}V, Percent: {battery_percent}%")
                except AttributeError as e:
                    print("Battery module error:", e)
                    battery_percent = 0
                battery_state = None  # Reset for next reading
            if wifi_ok and time.ticks_diff(now, last_time_sync) >= TIME_SYNC_INTERVAL:
                sync_time()
            if current_page == 2 and wifi_ok and time.ticks_diff(now, last_weather_update) >= WEATHER_UPDATE_INTERVAL:
                update_weather()
            if current_page == 3:
                read_modbus()
            if wifi_ok and time.ticks_diff(now, last_thingspeak_update) >= THINGSPEAK_INTERVAL:
                print("Uploading to ThingSpeak")
                upload_to_thingspeak(temp, hum, pressure, wind_speed)
                last_thingspeak_update = now
            last_main_update = now

        if battery_percent is not None and battery_percent <= 10 and not is_warning_active:
            if now >= next_warning_time:
                blink_warning()
                next_warning_time = now + 30000

        if oled_on and not is_warning_active:
            display_data()

if __name__ == "__main__":
    main()

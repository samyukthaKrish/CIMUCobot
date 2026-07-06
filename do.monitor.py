import requests
import time
import csv
from datetime import datetime

# --- CONFIG ---
DUET_IP = "172.20.10.3"
SENSOR_INDEX = 2
WATER_TEMP_C = 25
CAL1_V = 1600
CAL1_T = 25
THRESHOLD_MG_L = 8.0
POLL_INTERVAL = 2
LOG_FILE = "do_log.csv"

DO_TABLE = [
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080,  9860,  9660,  9460,  9270,
     9080,  8900,  8730,  8570,  8410,  8250,  8110,  7960,  7820,  7690,
     7560,  7430,  7300,  7180,  7070,  6950,  6840,  6730,  6630,  6530, 6410
]

def login():
    url = f"http://{DUET_IP}/rr_connect?password=C0sm0s8!"
    response = requests.get(url, timeout=5)
    print(f"Login: {response.text}")

def get_raw_voltage_mv():
    url = f"http://{DUET_IP}/rr_model?key=sensors.analog[{SENSOR_INDEX}]"
    response = requests.get(url, timeout=5)
    data = response.json()
    voltage_0_to_1 = data["result"]["lastReading"]
    return voltage_0_to_1 * 3000

def convert_to_mg_l(voltage_mv, temp_c):
    temp_c = max(0, min(40, int(temp_c)))
    v_saturation = CAL1_V + 35 * temp_c - CAL1_T * 35
    do_ug_l = voltage_mv * DO_TABLE[temp_c] / v_saturation
    return do_ug_l / 1000

def get_sat_percent(do_mg_l, temp_c):
    temp_c = max(0, min(40, int(temp_c)))
    max_mg_l = DO_TABLE[temp_c] / 1000
    return (do_mg_l / max_mg_l) * 100

def update_paneldue(do_mg_l, sat_percent):
    msg = f'M117 "DO: {do_mg_l:.2f} mg/L | {sat_percent:.1f}%"'
    url = f"http://{DUET_IP}/rr_gcode?gcode={requests.utils.quote(msg)}"
    requests.get(url, timeout=5)

def stop_pump():
    url = f"http://{DUET_IP}/rr_gcode?gcode=M42%20P0%20S0"
    requests.get(url, timeout=5)
    msg = 'M117 "DO THRESHOLD REACHED — PUMP OFF"'
    url2 = f"http://{DUET_IP}/rr_gcode?gcode={requests.utils.quote(msg)}"
    requests.get(url2, timeout=5)
    print("PUMP STOPPED — DO threshold reached")

# --- MAIN LOOP ---
with open(LOG_FILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "voltage_mv", "do_mg_l", "sat_percent"])

    login()
    print("Monitoring DO... press Ctrl+C to stop")
    while True:
        try:
            voltage_mv = get_raw_voltage_mv()
            do_mg_l = convert_to_mg_l(voltage_mv, WATER_TEMP_C)
            sat_percent = get_sat_percent(do_mg_l, WATER_TEMP_C)
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            print(f"[{timestamp}] Voltage: {voltage_mv:.1f}mV | DO: {do_mg_l:.2f} mg/L | Sat: {sat_percent:.1f}%")
            writer.writerow([timestamp, round(voltage_mv, 1), round(do_mg_l, 3), round(sat_percent, 1)])
            f.flush()

            update_paneldue(do_mg_l, sat_percent)

            if do_mg_l >= THRESHOLD_MG_L:
                # stop_pump()
                print("THRESHOLD REACHED — pump would stop here")
                break

        except Exception as e:
            print(f"Error: {e}")

        time.sleep(POLL_INTERVAL)
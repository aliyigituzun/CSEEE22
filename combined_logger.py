import csv
import threading
import time
from datetime import datetime

import serial
import paho.mqtt.client as mqtt

# ===================== USER CONFIG =====================
# Set these to the actual serial ports of your two boards.
# Examples:
#   macOS/Linux: "/dev/tty.usbserial-XXXXX" or "/dev/ttyUSB0"
#   Windows:     "COM3", "COM4", ...
PORTS = [
    "/dev/ttyUSB0",  # Board 1 (heating + motor)
    "/dev/ttyUSB1",  # Board 2 (if also using Serial)
]

BAUDRATE = 115200  # Must match Serial.begin(...) in your sketches (for any boards you log via Serial)
LOG_FILE = "logs.csv"
LOG_INTERVAL_SEC = 1.0

# MQTT broker where the pH controller is publishing
MQTT_BROKER = "engf0001.cs.ucl.ac.uk"
MQTT_PORT = 1883
MQTT_TOPIC_PH_STATE = "reactor/state/ph"


# ===================== STATE =====================
# We maintain the latest parsed values from each subsystem.
state_lock = threading.Lock()
state = {
    "heating": {
        "temp_C": None,
        "target_C": None,
        "voltage_V": None,
        "adc": None,
        "resistance_ohm": None,
        "heater_on": None,
    },
    "motor": {
        "measspeed_rpm": None,
        "Vmotor_raw": None,  # not available from current sketch, kept for future
    },
    "ph": {
        "ph_value": None,
        # setpoint and pump_on are not printed by current sketch, so we cannot log them
    },
}


# ===================== PARSERS =====================

def try_parse_float(text):
    try:
        return float(text)
    except Exception:
        return None


def parse_heating_line(line: str):
    """Update heating state from heating_control_v4.0 Serial output.

    Expected lines include for example:
      "Target: 20°C"
      "Temperature: 21.34°C"
      "Voltage: 3.12 V"
      "ADC: 1234"
      "Resistance: 10000 Ω"
      "Setting HEATER_PIN to HIGH" / "... Low"
    """
    line = line.strip()
    if line.startswith("Target:"):
        # e.g. "Target: 20°C"
        parts = line.split()
        if len(parts) >= 2:
            val = parts[1].replace("°C", "")
            with state_lock:
                state["heating"]["target_C"] = try_parse_float(val)

    elif line.startswith("Temperature:"):
        # e.g. "Temperature: 21.34°C"
        parts = line.split()
        if len(parts) >= 2:
            val = parts[1].replace("°C", "")
            with state_lock:
                state["heating"]["temp_C"] = try_parse_float(val)

    elif line.startswith("Voltage:"):
        # e.g. "Voltage: 3.12 V" (note colon)
        parts = line.split()
        if len(parts) >= 2:
            val = parts[1]
            with state_lock:
                state["heating"]["voltage_V"] = try_parse_float(val)

    elif line.startswith("ADC:"):
        parts = line.split()
        if len(parts) >= 2:
            try:
                adc_val = int(parts[1])
            except Exception:
                adc_val = None
            with state_lock:
                state["heating"]["adc"] = adc_val

    elif line.startswith("Resistance:"):
        # e.g. "Resistance: 10000 Ω"
        parts = line.split()
        if len(parts) >= 2:
            val = parts[1]
            with state_lock:
                state["heating"]["resistance_ohm"] = try_parse_float(val)

    elif "Setting HEATER_PIN to" in line:
        # "Setting HEATER_PIN to HIGH" or "Low"
        on = "HIGH" in line.upper()
        with state_lock:
            state["heating"]["heater_on"] = on


def parse_motor_line(line: str):
    """Update motor state from motor_control Serial output.

    Current sketch prints lines like:
      "0,1000,123.45"
    where the last field is meanmeasspeed (RPM).
    """
    line = line.strip()
    if "," not in line:
        return
    parts = line.split(",")
    if len(parts) < 3:
        return
    rpm_val = try_parse_float(parts[-1])
    if rpm_val is None:
        return
    with state_lock:
        state["motor"]["measspeed_rpm"] = rpm_val


def parse_ph_line(line: str):
    """Update pH state from ph_control Serial output.

    We look for lines like:
      "Voltage = 1.234 V, pH = 7.89"
    """
    # We now prefer MQTT for pH, but keep this as a fallback in case
    # you still print pH lines over Serial in the future.
    line = line.strip()
    if not line.startswith("Voltage ="):
        return
    try:
        first, second = line.split(",")
        ph_parts = second.split("=")
        if len(ph_parts) >= 2:
            ph_val = ph_parts[1].strip()
            ph = try_parse_float(ph_val)
        else:
            ph = None
        with state_lock:
            if ph is not None:
                state["ph"]["ph_value"] = ph
    except Exception:
        return


def parse_line(line: str):
    """Try all parsers on a single incoming line."""
    # Order matters slightly; heating and pH both mention voltage,
    # but we distinguish using the different text formats.
    parse_heating_line(line)
    parse_motor_line(line)
    parse_ph_line(line)


# ===================== SERIAL READER THREAD =====================

def reader_thread(port_name: str):
    print(f"[reader] Opening {port_name} at {BAUDRATE} baud...")
    try:
        with serial.Serial(port_name, BAUDRATE, timeout=1) as ser:
            print(f"[reader] {port_name} opened.")
            while True:
                try:
                    raw = ser.readline()
                    if not raw:
                        continue
                    try:
                        line = raw.decode(errors="ignore").strip()
                    except Exception:
                        continue
                    if line:
                        parse_line(line)
                except serial.SerialException as e:
                    print(f"[reader] Serial error on {port_name}: {e}")
                    break
    except serial.SerialException as e:
        print(f"[reader] Could not open {port_name}: {e}")


# ===================== CSV LOGGER =====================

CSV_HEADER = [
    "time_iso",
    # Heating
    "heat_temp_C",
    "heat_target_C",
    "heat_voltage_V",
    "heat_adc",
    "heat_resistance_ohm",
    "heat_heater_on",
    # Motor
    "motor_measspeed_rpm",
    "motor_Vmotor_raw",
    # pH
    "ph_value",
]


def ensure_log_file_has_header(path: str):
    try:
        with open(path, "r", newline="") as f:
            # If file exists and has any content, do nothing.
            if f.read(1):
                return
    except FileNotFoundError:
        pass
    # Create file with header
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)


def snapshot_state():
    with state_lock:
        s = {
            "heating": state["heating"].copy(),
            "motor": state["motor"].copy(),
            "ph": state["ph"].copy(),
        }
    return s


def main():
    ensure_log_file_has_header(LOG_FILE)

    # Start reader threads for each configured port
    threads = []
    for p in PORTS:
        t = threading.Thread(target=reader_thread, args=(p,), daemon=True)
        t.start()
        threads.append(t)

    # Set up MQTT client for pH data
    client = mqtt.Client()

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("[mqtt] Connected to broker, subscribing to", MQTT_TOPIC_PH_STATE)
            client.subscribe(MQTT_TOPIC_PH_STATE)
        else:
            print("[mqtt] Connection failed with code", rc)

    def on_message(client, userdata, msg):
        if msg.topic == MQTT_TOPIC_PH_STATE:
            try:
                payload = msg.payload.decode().strip()
            except Exception:
                return
            ph_val = try_parse_float(payload)
            with state_lock:
                state["ph"]["ph_value"] = ph_val

    client.on_connect = on_connect
    client.on_message = on_message
    print(f"[mqtt] Connecting to {MQTT_BROKER}:{MQTT_PORT} ...")
    client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    client.loop_start()

    print("[logger] Logging to", LOG_FILE)
    print("[logger] Press Ctrl+C to stop.")

    try:
        while True:
            time.sleep(LOG_INTERVAL_SEC)
            s = snapshot_state()
            row = [
                datetime.utcnow().isoformat(),
                # Heating
                s["heating"]["temp_C"],
                s["heating"]["target_C"],
                s["heating"]["voltage_V"],
                s["heating"]["adc"],
                s["heating"]["resistance_ohm"],
                s["heating"]["heater_on"],
                # Motor
                s["motor"]["measspeed_rpm"],
                s["motor"]["Vmotor_raw"],
                # pH
                s["ph"]["ph_value"],
            ]
            with open(LOG_FILE, "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(row)
    except KeyboardInterrupt:
        print("\n[logger] Stopped by user.")


if __name__ == "__main__":
    main()

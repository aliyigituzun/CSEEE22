import csv
import threading
import time
from datetime import datetime

import paho.mqtt.client as mqtt

LOG_FILE = "logs.csv"
LOG_INTERVAL_SEC = 1.0

MQTT_BROKER = "" 
MQTT_PORT = 8883
MQTT_USERNAME = "CSEEE22"
MQTT_PASSWORD = "Team22thebest"

MQTT_TOPIC_PH   = "reactor/state/ph"
MQTT_TOPIC_TEMP = "reactor/state/temp"
MQTT_TOPIC_RPM  = "reactor/state/rpm"

state_lock = threading.Lock()
state = {
    "ph": None,
    "temp_C": None,
    "rpm": None,
}

def try_parse_float(text):
    try:
        return float(text)
    except Exception:
        return None


CSV_HEADER = [
    "time_iso",
    "ph",
    "temp_C",
    "rpm",
]


def ensure_log_file_has_header(path: str):
    try:
        with open(path, "r", newline="") as f:
            if f.read(1):
                return
    except FileNotFoundError:
        pass

    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)


def snapshot_state():
    with state_lock:
        return dict(state)

def main():
    ensure_log_file_has_header(LOG_FILE)

    client = mqtt.Client()

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("[mqtt] Connected to broker")
            print("[mqtt] Subscribing to:", MQTT_TOPIC_PH, MQTT_TOPIC_TEMP, MQTT_TOPIC_RPM)
            client.subscribe([
                (MQTT_TOPIC_PH, 0),
                (MQTT_TOPIC_TEMP, 0),
                (MQTT_TOPIC_RPM, 0),
            ])
        else:
            print("[mqtt] Connection failed with code", rc)

    def on_message(client, userdata, msg):
        payload_str = None
        try:
            payload_str = msg.payload.decode().strip()
        except Exception:
            return

        val = try_parse_float(payload_str)
        if val is None:
            return

        with state_lock:
            if msg.topic == MQTT_TOPIC_PH:
                state["ph"] = val
            elif msg.topic == MQTT_TOPIC_TEMP:
                state["temp_C"] = val
            elif msg.topic == MQTT_TOPIC_RPM:
                state["rpm"] = val

    client.on_connect = on_connect
    client.on_message = on_message
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

    print(f"[mqtt] Connecting to {MQTT_BROKER}:{MQTT_PORT} ...")
    client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    client.loop_start()

    print("[logger] Logging pH, temperature, and RPM to", LOG_FILE)
    print("[logger] Press Ctrl+C to stop.")

    try:
        while True:
            time.sleep(LOG_INTERVAL_SEC)
            s = snapshot_state()
            row = [
                datetime.utcnow().isoformat(),
                s["ph"],
                s["temp_C"],
                s["rpm"],
            ]
            with open(LOG_FILE, "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(row)
    except KeyboardInterrupt:
        print("\n[logger] Stopped by user.")
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()

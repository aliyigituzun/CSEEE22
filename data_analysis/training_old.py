#!/usr/bin/env python3

import json
import signal
import sys
import threading
from datetime import datetime
import pickle

import pandas as pd
from paho.mqtt import client as mqtt
from statsmodels.tsa.arima.model import ARIMA

MQTT_BROKER = "engf0001.cs.ucl.ac.uk"
MQTT_PORT = 1883
MQTT_TOPIC = "bioreactor_sim/nofaults/telemetry/summary"

TRAIN_SAMPLES = 600  # ~10 minutes

VARIABLES = {
    "temperature_C": (2, 1, 2),
    "pH": (2, 1, 2),
    "rpm": (2, 1, 2),
}

MODEL_PATH = "arima_models.pkl"


data_lock = threading.Lock()
timestamps = []
values = {v: [] for v in VARIABLES} 

training_done_event = threading.Event()
shutdown_event = threading.Event()


def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("Connected to MQTT broker.")
        client.subscribe(MQTT_TOPIC)
        print(f"Subscribed to topic: {MQTT_TOPIC}")
    else:
        print(f"Failed to connect, return code {rc}")


def on_message(client, userdata, msg):
    global timestamps, values

    try:
        payload = msg.payload.decode("utf-8")
        data = json.loads(payload)
    except Exception as e:
        print(f"Error decoding JSON: {e}")
        return

    try:
        window_end = data["window"]["end"]
        ts = datetime.fromtimestamp(window_end)

        sample = {}
        for var in VARIABLES.keys():
            sample[var] = float(data[var]["mean"])

        with data_lock:
            timestamps.append(ts)
            for var, val in sample.items():
                values[var].append(val)

            n = len(timestamps)

        print(
            f"[{n:4d}] {ts}  "
            f"T={sample['temperature_C']:.3f} °C, "
            f"pH={sample['pH']:.3f}, "
            f"RPM={sample['rpm']:.2f}"
        )

        if n >= TRAIN_SAMPLES and not training_done_event.is_set():
            training_done_event.set()

    except KeyError as e:
        print(f"Missing expected key in JSON: {e}")
    except Exception as e:
        print(f"Unexpected error processing message: {e}")


def train_arima_models():
    with data_lock:
        n_samples = len(timestamps)
        if n_samples < 5:
            raise RuntimeError("Not enough data collected.")

        idx = pd.to_datetime(timestamps)
        df = pd.DataFrame(index=idx)
        for var in VARIABLES.keys():
            df[var] = values[var]

        df = df.sort_index()

        df = df[~df.index.duplicated(keep='first')]
        df = df.asfreq('1s')
        df = df.interpolate(method='linear')

    print("\nTraining ARIMA models on fault-free data...")
    print(f"Number of samples: {len(df)}")

    fitted_models = {}

    for var, order in VARIABLES.items():
        series = df[var]
        print(f"\n--- Training ARIMA{order} for {var} ---")
        
        model = ARIMA(series, order=order, freq='1s')
        fitted = model.fit()
        fitted_models[var] = fitted

        print(f"Model for {var} fitted. AIC: {fitted.aic:.2f}")

    with open(MODEL_PATH, "wb") as f:
        pickle.dump(fitted_models, f)

    print(f"\nSaved fitted ARIMA models to '{MODEL_PATH}'.")


def handle_sigint(signum, frame):
    print("\nSIGINT received, shutting down...")
    shutdown_event.set()
    training_done_event.set()


def main():
    signal.signal(signal.SIGINT, handle_sigint)

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    print(f"Connecting to {MQTT_BROKER}...")
    client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)

    client.loop_start()

    print(f"Collecting {TRAIN_SAMPLES} samples...")
    training_done_event.wait()

    client.loop_stop()
    client.disconnect()

    if shutdown_event.is_set() and len(timestamps) < TRAIN_SAMPLES:
        print("Interrupted. Exiting.")
        sys.exit(1)

    try:
        train_arima_models()
    except Exception as e:
        print(f"Error during ARIMA training: {e}")
        sys.exit(1)

    print("\nDone.")


if __name__ == "__main__":
    main()
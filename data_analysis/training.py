#!/usr/bin/env python3
"""
Train ARIMA models on fault-free ENGF0001 bioreactor data streamed over MQTT.

This script subscribes to the 'nofaults' stream, collects temperature, pH and
RPM, and fits a separate ARIMA model to each of these time series.

Requirements:
    pip install paho-mqtt statsmodels pandas

Network note:
    You must be on the UCL network or connected via UCL VPN to reach
    engf0001.cs.ucl.ac.uk:1883.
"""

import json
import signal
import sys
import threading
from datetime import datetime
import pickle

import pandas as pd
from paho.mqtt import client as mqtt
from statsmodels.tsa.arima.model import ARIMA

# ==========================
# CONFIG
# ==========================

MQTT_BROKER = "engf0001.cs.ucl.ac.uk"
MQTT_PORT = 1883
MQTT_TOPIC = "bioreactor_sim/nofaults/telemetry/summary"

# One message per second; 600 ≈ 10 minutes
TRAIN_SAMPLES = 600

VARIABLES = {
    "temperature_C": (2, 1, 2),
    "pH": (2, 1, 2),
    "rpm": (2, 1, 2),
}

MODEL_PATH = "arima_models.pkl"


# ==========================
# GLOBALS / STATE
# ==========================

data_lock = threading.Lock()
timestamps = []                     
values = {v: [] for v in VARIABLES} 

training_done_event = threading.Event()
shutdown_event = threading.Event()


# ==========================
# MQTT CALLBACKS
# ==========================

def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("Connected to MQTT broker.")
        client.subscribe(MQTT_TOPIC)
        print(f"Subscribed to topic: {MQTT_TOPIC}")
    else:
        print(f"Failed to connect, return code {rc}")


def on_message(client, userdata, msg):
    """Handle incoming telemetry JSON and extract the training features."""
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


# ==========================
# TRAINING FUNCTION
# ==========================

def train_arima_models():
    """Fit one ARIMA model per variable (temperature_C, pH, rpm)."""
    with data_lock:
        n_samples = len(timestamps)
        if n_samples < 5:
            raise RuntimeError("Not enough data collected.")

        idx = pd.to_datetime(timestamps)
        df = pd.DataFrame(index=idx)
        for var in VARIABLES.keys():
            df[var] = values[var]

        df = df.sort_index()

        ### add interpolation to handle MQTT network errors ###
        # 1. Drop duplicates (in case of network retries)
        df = df[~df.index.duplicated(keep='first')]
        
        # 2. Force 1-second frequency and fill missing gaps (Critical for ARIMA)
        df = df.asfreq('1s')
        df = df.interpolate(method='linear')
        # ### IMPROVEMENT END ###

    print("\nTraining ARIMA models on fault-free data...")
    print(f"Number of samples: {len(df)}") # Updated length after resampling

    fitted_models = {}

    for var, order in VARIABLES.items():
        series = df[var]
        print(f"\n--- Training ARIMA{order} for {var} ---")
        
        # ### IMPROVEMENT: Pass freq='1s' explicitly to suppress warnings
        model = ARIMA(series, order=order, freq='1s')
        fitted = model.fit()
        fitted_models[var] = fitted

        print(f"Model for {var} fitted. AIC: {fitted.aic:.2f}")

    with open(MODEL_PATH, "wb") as f:
        pickle.dump(fitted_models, f)

    print(f"\nSaved fitted ARIMA models to '{MODEL_PATH}'.")


# ==========================
# CLEAN SHUTDOWN HANDLER
# ==========================

def handle_sigint(signum, frame):
    print("\nSIGINT received, shutting down...")
    shutdown_event.set()
    training_done_event.set()


# ==========================
# MAIN
# ==========================

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
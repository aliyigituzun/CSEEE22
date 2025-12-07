#!/usr/bin/env python3
"""
Check MQTT connection and ARIMA pipeline using trained models.

This script:
  - loads previously trained ARIMA models for temperature_C, pH and rpm
  - connects to a specified MQTT broker/topic
  - parses incoming JSON telemetry
  - prints ARIMA one-step predictions and residuals for each variable

Requirements:
    pip install paho-mqtt statsmodels pandas
"""

import json
import signal
import sys
import threading
from datetime import datetime
import pickle

import pandas as pd
from paho.mqtt import client as mqtt

# ==========================
# CONFIG — FILL THESE IN
# ==========================

# <<< REPLACE these with YOUR connection details >>>
MQTT_BROKER = "http://b6dbb6381f0c4bbf8360ef22013ff085.s1.eu.hivemq.cloud/"
MQTT_PORT = 8883
MQTT_USERNAME = "CSEEE22"  
MQTT_PASSWORD = "Team22thebest"

# Topic you want to test (e.g. a faulted stream, or your own topic)
MQTT_TOPIC_TEST = "your/topic/here"

# Path to the models we trained earlier (dict: {var_name: fitted_model})
MODEL_PATH = "arima_models.pkl"


# ==========================
# GLOBALS / STATE
# ==========================

models = {}           # var -> ARIMAResults
time_indices = {}     # var -> next integer time index for prediction
state_lock = threading.Lock()
shutdown_event = threading.Event()


# ==========================
# LOAD MODELS
# ==========================

def load_models():
    global models, time_indices

    with open(MODEL_PATH, "rb") as f:
        models = pickle.load(f)

    # Initialise "time index" for predictions to end of training data
    # Statsmodels stores the original endog length; we'll just advance from there.
    for var, fitted in models.items():
        nobs = len(fitted.data.endog)
        time_indices[var] = nobs  # next prediction index
        print(f"Loaded model for {var}, training length = {nobs}")

    expected = {"temperature_C", "pH", "rpm"}
    missing = expected - set(models.keys())
    if missing:
        print(f"WARNING: models missing for {missing}")


# ==========================
# MQTT CALLBACKS
# ==========================

def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("Connected to MQTT broker.")
        client.subscribe(MQTT_TOPIC_TEST)
        print(f"Subscribed to topic: {MQTT_TOPIC_TEST}")
    else:
        print(f"Failed to connect, return code {rc}")


def predict_and_residual(var_name, y_t):
    """
    Use the loaded ARIMA model for var_name to produce a one-step prediction
    and residual. This uses statsmodels' internal indexing rather than
    fully updating the model state with each new observation, but is fine
    for checking the pipeline/connection.
    """
    fitted = models[var_name]
    with state_lock:
        idx = time_indices[var_name]
        # one-step prediction at the "next" index
        y_hat = float(fitted.predict(start=idx, end=idx)[0])
        time_indices[var_name] += 1

    residual = float(y_t) - y_hat
    return y_hat, residual


def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode("utf-8")
        data = json.loads(payload)
    except Exception as e:
        print(f"Error decoding JSON: {e}")
        return

    try:
        window_end = data["window"]["end"]
        ts = datetime.fromtimestamp(window_end)

        temp = float(data["temperature_C"]["mean"])
        ph = float(data["pH"]["mean"])
        rpm = float(data["rpm"]["mean"])

        # Get predictions and residuals
        temp_hat, temp_res = predict_and_residual("temperature_C", temp)
        ph_hat, ph_res = predict_and_residual("pH", ph)
        rpm_hat, rpm_res = predict_and_residual("rpm", rpm)

        # Pretty print one line per message
        print(
            f"{ts} | "
            f"T: obs={temp:6.3f}, pred={temp_hat:6.3f}, res={temp_res:+7.3f} | "
            f"pH: obs={ph:5.3f}, pred={ph_hat:5.3f}, res={ph_res:+6.3f} | "
            f"RPM: obs={rpm:7.2f}, pred={rpm_hat:7.2f}, res={rpm_res:+8.2f}"
        )

    except KeyError as e:
        print(f"Missing expected key in JSON: {e}")
    except Exception as e:
        print(f"Unexpected error processing message: {e}")


# ==========================
# CLEAN SHUTDOWN HANDLER
# ==========================

def handle_sigint(signum, frame):
    print("\nSIGINT received, shutting down...")
    shutdown_event.set()


# ==========================
# MAIN
# ==========================

def main():
    signal.signal(signal.SIGINT, handle_sigint)

    # 1. Load the trained models
    try:
        load_models()
    except Exception as e:
        print(f"Failed to load ARIMA models from '{MODEL_PATH}': {e}")
        sys.exit(1)

    # 2. Configure MQTT client
    client = mqtt.Client()
    if MQTT_USERNAME is not None:
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

    client.on_connect = on_connect
    client.on_message = on_message

    print(f"Connecting to MQTT broker {MQTT_BROKER}:{MQTT_PORT} ...")
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    except Exception as e:
        print(f"Error connecting to MQTT broker: {e}")
        sys.exit(1)

    client.loop_start()
    print("Listening for messages (Ctrl+C to stop)...")

    # 3. Just sit in a loop until user stops
    try:
        while not shutdown_event.is_set():
            shutdown_event.wait(timeout=1.0)
    finally:
        client.loop_stop()
        client.disconnect()
        print("Disconnected. Bye.")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Evaluate SARIMAX-based anomaly detector on faulted streams.

- Loads trained model bundle from sarimax_models_nofaults.pkl
- Subscribes to e.g. bioreactor_sim/three_faults/telemetry/summary
- For each sample:
    * builds exogenous features (setpoints, PWMs, dosing, etc.)
    * computes standardized residuals per variable
    * computes joint score S = sum(z_i^2)
    * flags anomaly if S > chi2_threshold (from training)

- Compares anomaly flag against ground truth fault label:
    * y_true = 1 if any faults are active (faults.last_active non-empty), else 0
- Tracks TP, TN, FP, FN and prints:
    * precision, recall, F1, accuracy

Requirements:
    pip install paho-mqtt statsmodels pandas numpy
"""

import json
import signal
import sys
import threading
from datetime import datetime
import pickle
import warnings

import numpy as np
import pandas as pd
from paho.mqtt import client as mqtt

# Silence some of the spammy statsmodels warnings
warnings.filterwarnings("ignore", category=UserWarning)
warnings.filterwarnings("ignore", category=FutureWarning)

# ==========================
# CONFIG
# ==========================

MQTT_BROKER = "engf0001.cs.ucl.ac.uk"
MQTT_PORT = 1883

# Choose which stream to evaluate on:
#   "single_fault" or "three_faults" or "variable_setpoints"
DATA_STREAM = "single_fault"

MQTT_TOPIC = f"bioreactor_sim/{DATA_STREAM}/telemetry/summary"

MODEL_PATH = "arima_models.pkl"

VARIABLES = ["temperature_C", "pH", "rpm"]

# Minimum history length before we start evaluating (in samples)
MIN_HISTORY = 30


# ==========================
# GLOBAL STATE
# ==========================

data_lock = threading.Lock()

timestamps_eval = []                     # list[datetime]
values_eval = {v: [] for v in VARIABLES} # dict[var] -> list[float]
exog_eval = []                           # list[dict exog_name -> float]

# model bundle loaded from pickle
model_bundle = None
exog_columns = []

# Confusion matrix counts
TP = 0
FP = 0
TN = 0
FN = 0
N_SAMPLES = 0

shutdown_event = threading.Event()


# ==========================
# HELPERS
# ==========================

def build_exog_sample(data):
    """
    Build exogenous feature dict from JSON payload.
    This MUST match the exog definition used in the training script.
    Adjust here if you changed it there.
    """
    exog_sample = {
        "temp_setpoint":   float(data["setpoints"]["temperature_C"]),
        "ph_setpoint":     float(data["setpoints"]["pH"]),
        "rpm_setpoint":    float(data["setpoints"]["rpm"]),

        "heater_pwm":      float(data["actuators_avg"]["heater_pwm"]),
        "motor_pwm":       float(data["actuators_avg"]["motor_pwm"]),
        "acid_pwm":        float(data["actuators_avg"]["acid_pwm"]),
        "base_pwm":        float(data["actuators_avg"]["base_pwm"]),

        "heater_energy_Wh": float(data["heater_energy_Wh"]),
        "photoevents":      float(data["photoevents"]),

        "acid_dosed_l":    float(data["dosing_l"]["acid"]),
        "base_dosed_l":    float(data["dosing_l"]["base"]),
    }
    return exog_sample


def compute_anomaly_and_score():
    """
    Using all collected data so far:
      - construct pandas Series/DataFrames
      - apply each SARIMAX model to compute one-step-ahead prediction
      - compute residuals + z-scores
      - compute joint score S = sum(z_i^2)
      - compare to chi2_threshold from training

    Returns:
        anomaly (bool), S (float), per_var_z (dict[var -> z-score])
        or (None, None, None) if not enough history yet
    """
    global model_bundle, exog_columns

    with data_lock:
        n = len(timestamps_eval)
        if n < MIN_HISTORY:
            return None, None, None

        idx = pd.to_datetime(timestamps_eval)
        df = pd.DataFrame(index=idx)

        for var in VARIABLES:
            df[var] = values_eval[var]

        if exog_columns:
            exog_df = pd.DataFrame(exog_eval, index=idx)
            # keep only the columns that were used during training, in same order
            exog_df = exog_df[exog_columns]
        else:
            exog_df = None

    # sort by time just in case
    df = df.sort_index()
    if exog_df is not None:
        exog_df = exog_df.sort_index()


    S = 0.0
    per_var_z = {}

    for var in VARIABLES:
        series = df[var]
        info = model_bundle["models"][var]
        fitted = info["fitted"]
        res_mean = info["res_mean"]
        res_std = info["res_std"] if info["res_std"] > 0 else 1.0

        # Re-apply the fitted parameters to the current data to get states
        if exog_df is not None:
            applied = fitted.apply(endog=series, exog=exog_df)
            pred_res = applied.get_prediction(
                start=len(series) - 1,
                end=len(series) - 1,
                exog=exog_df,
            )
        else:
            applied = fitted.apply(endog=series)
            pred_res = applied.get_prediction(
                start=len(series) - 1,
                end=len(series) - 1,
            )

        y_hat = float(pred_res.predicted_mean.iloc[-1])
        y_true_last = float(series.iloc[-1])
        resid = y_true_last - y_hat

        z = (resid - res_mean) / res_std
        per_var_z[var] = z
        S += z ** 2

    chi2_threshold = model_bundle["chi2_threshold"]
    anomaly = S > chi2_threshold

    return anomaly, S, per_var_z


def print_final_scores():
    global TP, FP, TN, FN, N_SAMPLES

    print("\n================ Evaluation Summary ================")
    print(f"Samples processed: {N_SAMPLES}")
    print(f"TP = {TP}, FP = {FP}, TN = {TN}, FN = {FN}")

    if TP + FP > 0:
        precision = TP / (TP + FP)
    else:
        precision = 0.0

    if TP + FN > 0:
        recall = TP / (TP + FN)
    else:
        recall = 0.0

    if precision + recall > 0:
        f1 = 2 * precision * recall / (precision + recall)
    else:
        f1 = 0.0

    if N_SAMPLES > 0:
        accuracy = (TP + TN) / N_SAMPLES
    else:
        accuracy = 0.0

    print(f"Precision: {precision:.4f}")
    print(f"Recall:    {recall:.4f}")
    print(f"F1-score:  {f1:.4f}")
    print(f"Accuracy:  {accuracy:.4f}")
    print("===================================================\n")


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
    global TP, FP, TN, FN, N_SAMPLES

    try:
        payload = msg.payload.decode("utf-8")
        data = json.loads(payload)
    except Exception as e:
        print(f"Error decoding JSON: {e}")
        return

    try:
        window_end = data["window"]["end"]
        ts = datetime.fromtimestamp(window_end)

        # Primary variables
        sample = {}
        for var in VARIABLES:
            sample[var] = float(data[var]["mean"])

        # Exogenous features
        exog_sample = build_exog_sample(data)

        # Ground-truth label: 1 if any fault active, else 0
        # faults.last_active is a list of active fault names
        faults_active = data.get("faults", {}).get("last_active", [])
        y_true = 1 if len(faults_active) > 0 else 0

        with data_lock:
            timestamps_eval.append(ts)
            for var, val in sample.items():
                values_eval[var].append(val)
            exog_eval.append(exog_sample)

        # Compute anomaly / score
        anomaly, S, per_var_z = compute_anomaly_and_score()

        if anomaly is None:
            # Not enough history yet
            print(f"[warmup] {ts}  T={sample['temperature_C']:.3f} °C, "
                  f"pH={sample['pH']:.3f}, RPM={sample['rpm']:.2f}, "
                  f"faults_active={faults_active}")
            return

        y_pred = 1 if anomaly else 0

        # Update confusion matrix
        if y_true == 1 and y_pred == 1:
            TP += 1
        elif y_true == 0 and y_pred == 1:
            FP += 1
        elif y_true == 0 and y_pred == 0:
            TN += 1
        elif y_true == 1 and y_pred == 0:
            FN += 1

        N_SAMPLES += 1

        # Nicely formatted print for each sample
        status = "ANOM" if anomaly else "OK"
        print(
            f"[{N_SAMPLES:5d}] {ts}  "
            f"T={sample['temperature_C']:.3f} °C, "
            f"pH={sample['pH']:.3f}, "
            f"RPM={sample['rpm']:.2f} | "
            f"S={S:.3f} | "
            f"z(T)={per_var_z['temperature_C']:.2f}, "
            f"z(pH)={per_var_z['pH']:.2f}, "
            f"z(RPM)={per_var_z['rpm']:.2f} | "
            f"faults={faults_active} | "
            f"pred={status}, true={'FAULT' if y_true else 'NOFAULT'}"
        )

    except KeyError as e:
        print(f"Missing expected key in JSON: {e}")
    except Exception as e:
        print(f"Unexpected error processing message: {e}")


# ==========================
# CLEAN SHUTDOWN HANDLER
# ==========================

def handle_sigint(signum, frame):
    print("\nSIGINT received, stopping evaluation...")
    shutdown_event.set()


# ==========================
# MAIN
# ==========================

def main():
    global model_bundle, exog_columns

    # Load trained model bundle
    print(f"Loading model bundle from '{MODEL_PATH}'...")
    with open(MODEL_PATH, "rb") as f:
        model_bundle = pickle.load(f)

    exog_columns = model_bundle.get("exog_columns", [])
    print("Loaded model for variables:", model_bundle["variables"])
    print("Exogenous columns:", exog_columns)
    print("Chi-square-like threshold:", model_bundle["chi2_threshold"])

    signal.signal(signal.SIGINT, handle_sigint)

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    print(f"Connecting to {MQTT_BROKER}...")
    client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)

    client.loop_start()

    print(f"Evaluating on stream '{DATA_STREAM}'. Press Ctrl+C to stop.\n")

    # Wait until Ctrl+C
    try:
        while not shutdown_event.is_set():
            shutdown_event.wait(timeout=1.0)
    finally:
        client.loop_stop()
        client.disconnect()

    print_final_scores()
    print("Done.")


if __name__ == "__main__":
    main()

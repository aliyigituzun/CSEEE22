#!/usr/bin/env python3
"""
Train SARIMAX (ARIMA+exogenous) models on fault-free ENGF0001 bioreactor data.

- Subscribes to the 'nofaults' stream over MQTT
- Collects temperature, pH and RPM
- Also collects setpoints & actuator duties as exogenous inputs
- Fits one SARIMAX model per variable with automatic (p, d, q) selection
- Saves:
    * fitted models
    * ARIMA order per variable
    * per-variable residual mean/std
    * a joint chi-square-like residual threshold

You can later load this pickle in your *detection* script and:
    - compute residual z-scores per variable
    - compute a joint score S = sum(z_i^2)
    - compare S to the stored chi2_threshold

Requirements:
    pip install paho-mqtt statsmodels pandas numpy
"""

import json
import signal
import sys
import threading
from datetime import datetime
import pickle

import numpy as np
import pandas as pd
from paho.mqtt import client as mqtt
from statsmodels.tsa.statespace.sarimax import SARIMAX

# ==========================
# CONFIG
# ==========================

MQTT_BROKER = "engf0001.cs.ucl.ac.uk"
MQTT_PORT = 1883
MQTT_TOPIC = "bioreactor_sim/nofaults/telemetry/summary"

# One message per second; 3600 ≈ 1 hour
TRAIN_SAMPLES = 3600

# Variables we model
VARIABLES = ["temperature_C", "pH", "rpm"]

# Candidate ARIMA orders (p, d, q) to search over for each variable
CANDIDATE_ORDERS = [
    (1, 0, 1),
    (1, 1, 1),
    (2, 0, 2),
    (2, 1, 2),
    (3, 1, 2),
]

MODEL_PATH = "sarimax_models_nofaults.pkl"


# ==========================
# GLOBALS / STATE
# ==========================

data_lock = threading.Lock()
timestamps = []                     # list of datetime objects
values = {v: [] for v in VARIABLES} # dict: var -> list of floats
exog_values = []                    # list of dicts (exogenous features per sample)

training_done_event = threading.Event()
shutdown_event = threading.Event()


# ==========================
# HELPERS
# ==========================

def nested_get(d, keys, default=np.nan):
    """
    Safely fetch nested values from JSON: nested_get(data, ("foo", "bar"))
    Returns default (np.nan) if any key is missing.
    """
    cur = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


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
    """Handle incoming telemetry JSON and extract training features."""
    global timestamps, values, exog_values

    try:
        payload = msg.payload.decode("utf-8")
        data = json.loads(payload)
    except Exception as e:
        print(f"Error decoding JSON: {e}")
        return

    try:
        # Timestamp from window end
        window_end = data["window"]["end"]
        ts = datetime.fromtimestamp(window_end)

        # --- primary variables (same as before) ---
        sample = {}
        for var in VARIABLES:  # ["temperature_C", "pH", "rpm"]
            sample[var] = float(data[var]["mean"])

        # --- exogenous inputs based on YOUR JSON structure ---
        # All of these are numeric in your example:
        #   "actuators_avg": {heater_pwm, motor_pwm, acid_pwm, base_pwm}
        #   "setpoints": {temperature_C, pH, rpm}
        #   "dosing_l": {acid, base}
        #   "heater_energy_Wh", "photoevents"
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

        with data_lock:
            timestamps.append(ts)
            for var, val in sample.items():
                values[var].append(val)
            exog_values.append(exog_sample)

            n = len(timestamps)

        # --- nicer print: all the main values in one line ---
        print(
            f"[{n:4d}] {ts}  "
            f"T={sample['temperature_C']:.3f} °C (sp={exog_sample['temp_setpoint']:.2f}), "
            f"pH={sample['pH']:.3f} (sp={exog_sample['ph_setpoint']:.2f}), "
            f"RPM={sample['rpm']:.2f} (sp={exog_sample['rpm_setpoint']:.1f}) | "
            f"Heater PWM={exog_sample['heater_pwm']:.3f}, "
            f"Motor PWM={exog_sample['motor_pwm']:.3f}, "
            f"Acid PWM={exog_sample['acid_pwm']:.5f}, "
            f"Base PWM={exog_sample['base_pwm']:.5f} | "
            f"Acid dose={exog_sample['acid_dosed_l']:.3e} L, "
            f"Base dose={exog_sample['base_dosed_l']:.3e} L | "
            f"Energy={exog_sample['heater_energy_Wh']:.5f} Wh, "
            f"Photoevents={exog_sample['photoevents']:.0f}"
        )

        if n >= TRAIN_SAMPLES and not training_done_event.is_set():
            training_done_event.set()

    except KeyError as e:
        print(f"Missing expected key in JSON: {e}")
    except Exception as e:
        print(f"Unexpected error processing message: {e}")

# ==========================
# TRAINING
# ==========================

def select_best_order_with_exog(series, exog, candidate_orders):
    """
    For one variable:
        - try each (p, d, q) in candidate_orders
        - fit SARIMAX with exog
        - return the model with the lowest AIC
    """
    best_aic = np.inf
    best_order = None
    best_model = None

    for order in candidate_orders:
        try:
            print(f"  Trying SARIMAX{order} ...")
            model = SARIMAX(
                series,
                order=order,
                exog=exog,
                freq="1s",
                enforce_stationarity=False,
                enforce_invertibility=False,
            )
            fitted = model.fit(disp=False)
            print(f"    AIC = {fitted.aic:.2f}")
            if fitted.aic < best_aic:
                best_aic = fitted.aic
                best_order = order
                best_model = fitted
        except Exception as e:
            print(f"    SARIMAX{order} failed: {e}")

    return best_order, best_model


def train_sarimax_models():
    """Fit one SARIMAX model per variable with exogenous features."""
    with data_lock:
        n_samples = len(timestamps)
        if n_samples < 5:
            raise RuntimeError("Not enough data collected.")

        idx = pd.to_datetime(timestamps)

        # --- main variables ---
        df = pd.DataFrame(index=idx)
        for var in VARIABLES:
            df[var] = values[var]

        # --- exogenous features ---
        exog_df = pd.DataFrame(exog_values, index=idx)

        # Sort indices
        df = df.sort_index()
        exog_df = exog_df.sort_index()

        # Drop duplicate timestamps
        df = df[~df.index.duplicated(keep="first")]
        exog_df = exog_df[~exog_df.index.duplicated(keep="first")]

        # === resample to 1 Hz and linearly interpolate ===
        df = df.asfreq("1s")
        df = df.interpolate(method="linear")

        exog_df = exog_df.asfreq("1s")

        # Replace ±inf with NaN first
        exog_df = exog_df.replace([np.inf, -np.inf], np.nan)

        # Drop exog columns that are entirely NaN (e.g. pumps never used)
        all_nan_cols = exog_df.columns[exog_df.isna().all()]
        if len(all_nan_cols) > 0:
            print(f"Dropping exog columns with all NaNs: {list(all_nan_cols)}")
            exog_df = exog_df.drop(columns=list(all_nan_cols))

        # If we dropped everything, just run *without* exogenous variables
        if exog_df.shape[1] == 0:
            print("WARNING: no valid exogenous columns left; training plain ARIMA models.")
            use_exog = False
            exog_df = None
        else:
            use_exog = True

            # Time interpolation first
            exog_df = exog_df.interpolate(method="time")

            # Forward/backward fill remaining gaps
            exog_df = exog_df.ffill().bfill()

        # Align df and exog_df indices (in case of any mismatch)
        if use_exog:
            df, exog_df = df.align(exog_df, join="inner", axis=0)
        else:
            # Ensure df has no NaNs left
            df = df.dropna(axis=0, how="any")

        # Clip extreme outliers in main variables (very conservative)
        df = df.clip(
            lower=df.quantile(0.001),
            upper=df.quantile(0.999),
            axis=1,
        )

        # Final safety: drop any rows with NaNs in either df or exog_df
        if use_exog:
            mask_bad = df.isna().any(axis=1) | exog_df.isna().any(axis=1)
            if mask_bad.any():
                print(f"Dropping {mask_bad.sum()} rows with remaining NaNs.")
                df = df[~mask_bad]
                exog_df = exog_df[~mask_bad]
        else:
            mask_bad = df.isna().any(axis=1)
            if mask_bad.any():
                print(f"Dropping {mask_bad.sum()} rows with remaining NaNs (no exog).")
                df = df[~mask_bad]

    print("\nTraining SARIMAX models on fault-free data...")
    print(f"Number of samples after cleaning: {len(df)}")
    print(f"Index start: {df.index[0]}, end: {df.index[-1]}")

    if use_exog:
        exog_columns = list(exog_df.columns)
        print(f"Using exogenous columns: {exog_columns}")
    else:
        exog_columns = []
        print("No exogenous columns in use (plain ARIMA).")

    model_bundle = {
        "variables": VARIABLES,
        "exog_columns": exog_columns,
        "models": {},             # per-variable model & stats
        "chi2_threshold": None,   # global joint threshold
    }

    # Store per-variable z-score series for joint chi-square
    z_residuals = {}

    for var in VARIABLES:
        print(f"\n=== Training model for {var} ===")
        series = df[var]

        if use_exog:
            exog_for_fit = exog_df
        else:
            exog_for_fit = None

        order, fitted = select_best_order_with_exog(series, exog_for_fit, CANDIDATE_ORDERS)
        if fitted is None:
            raise RuntimeError(f"Could not fit any SARIMAX/ARIMA model for {var}")

        print(f"Selected order for {var}: {order}, AIC={fitted.aic:.2f}")

        # In-sample prediction for residual stats
        pred = fitted.predict(start=0, end=len(series) - 1, exog=exog_for_fit)
        residuals = series - pred

        res_mean = float(residuals.mean())
        res_std = float(residuals.std(ddof=1))

        print(f"Residual mean for {var}: {res_mean:.6f}")
        print(f"Residual std  for {var}: {res_std:.6f}")

        z = (residuals - res_mean) / (res_std if res_std > 0 else 1.0)
        z_residuals[var] = z

        model_bundle["models"][var] = {
            "order": order,
            "res_mean": res_mean,
            "res_std": res_std,
            "fitted": fitted,
        }

    # Joint chi-square-like score across variables
    z_df = pd.DataFrame(z_residuals)
    S = (z_df ** 2).sum(axis=1)

    chi2_threshold = float(np.quantile(S, 0.99))
    model_bundle["chi2_threshold"] = chi2_threshold

    print(f"\nJoint chi-square-like score S: mean={S.mean():.3f}, std={S.std(ddof=1):.3f}")
    print(f"99th percentile threshold for S: {chi2_threshold:.3f}")

    # Save bundle
    with open(MODEL_PATH, "wb") as f:
        pickle.dump(model_bundle, f)

    print(f"\nSaved models + stats to '{MODEL_PATH}'.")
    """Fit one SARIMAX model per variable with exogenous features."""
    with data_lock:
        n_samples = len(timestamps)
        if n_samples < 5:
            raise RuntimeError("Not enough data collected.")

        idx = pd.to_datetime(timestamps)
        df = pd.DataFrame(index=idx)
        for var in VARIABLES:
            df[var] = values[var]

        exog_df = pd.DataFrame(exog_values, index=idx)

        # Sort and clean indices
        df = df.sort_index()
        exog_df = exog_df.sort_index()

        # Drop duplicates
        df = df[~df.index.duplicated(keep="first")]
        exog_df = exog_df[~exog_df.index.duplicated(keep="first")]

        # Force 1-second frequency and interpolate
        df = df.asfreq("1s")
        df = df.interpolate(method="linear")

        exog_df = exog_df.asfreq("1s")
        exog_df = exog_df.interpolate(method="linear")

        # Align in case of any slight mismatch
        df, exog_df = df.align(exog_df, join="inner", axis=0)

        # Clip extreme outliers (very conservative)
        df = df.clip(
            lower=df.quantile(0.001),
            upper=df.quantile(0.999),
            axis=1,
        )

    print("\nTraining SARIMAX models on fault-free data...")
    print(f"Number of samples after resampling/alignment: {len(df)}")
    print(f"Index start: {df.index[0]}, end: {df.index[-1]}")

    exog_columns = list(exog_df.columns)
    print(f"Using exogenous columns: {exog_columns}")

    model_bundle = {
        "variables": VARIABLES,
        "exog_columns": exog_columns,
        "models": {},             # per-variable model & stats
        "chi2_threshold": None,   # global joint threshold
    }

    # Store per-variable z-score series to build a joint chi-square statistic
    z_residuals = {}

    for var in VARIABLES:
        print(f"\n=== Training model for {var} ===")
        series = df[var]

        order, fitted = select_best_order_with_exog(series, exog_df, CANDIDATE_ORDERS)
        if fitted is None:
            raise RuntimeError(f"Could not fit any SARIMAX model for {var}")

        print(f"Selected order for {var}: {order}, AIC={fitted.aic:.2f}")

        # In-sample prediction for residual stats
        # Using full in-sample range
        pred = fitted.predict(start=0, end=len(series) - 1, exog=exog_df)
        residuals = series - pred

        res_mean = float(residuals.mean())
        res_std = float(residuals.std(ddof=1))

        print(f"Residual mean for {var}: {res_mean:.6f}")
        print(f"Residual std  for {var}: {res_std:.6f}")

        # Standardized residuals (z-scores)
        z = (residuals - res_mean) / (res_std if res_std > 0 else 1.0)
        z_residuals[var] = z

        model_bundle["models"][var] = {
            "order": order,
            "res_mean": res_mean,
            "res_std": res_std,
            # We store the fitted result object itself for later use
            "fitted": fitted,
        }

    # ---- joint chi-square-like threshold over variables ----
    z_df = pd.DataFrame(z_residuals)
    S = (z_df ** 2).sum(axis=1)  # S_t = sum_i z_{i,t}^2

    # Take 99th percentile as an anomaly threshold
    chi2_threshold = float(np.quantile(S, 0.99))
    model_bundle["chi2_threshold"] = chi2_threshold

    print(f"\nJoint chi-square-like score S: mean={S.mean():.3f}, std={S.std(ddof=1):.3f}")
    print(f"99th percentile threshold for S: {chi2_threshold:.3f}")

    # Save everything
    with open(MODEL_PATH, "wb") as f:
        pickle.dump(model_bundle, f)

    print(f"\nSaved SARIMAX models + stats to '{MODEL_PATH}'.")


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
        print("Interrupted before enough samples were collected. Exiting.")
        sys.exit(1)

    try:
        train_sarimax_models()
    except Exception as e:
        print(f"Error during SARIMAX training: {e}")
        sys.exit(1)

    print("\nDone.")


if __name__ == "__main__":
    main()

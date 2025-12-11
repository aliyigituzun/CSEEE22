import time
import random
import paho.mqtt.client as mqtt

# MQTT Configuration
BROKER = "" # MQTT URL
PORT = 1883
USERNAME = "" # MQTT Username
PASSWORD = "" # MQTT Password

# MQTT Topics
TOPIC_RPM_STATE  = "reactor/state/rpm"
TOPIC_PH_STATE   = "reactor/state/ph"
TOPIC_TEMP_STATE = "reactor/state/temp"

TOPIC_RPM_SETPOINT  = "reactor/setpoint/rpm"
TOPIC_PH_SETPOINT   = "reactor/setpoint/ph"
TOPIC_TEMP_SETPOINT = "reactor/setpoint/temp"

SETPOINT_RPM  = 800
SETPOINT_PH   = 7.0
SETPOINT_TEMP = 37.0

def on_connect(client, userdata, flags, rc):
    print("Connected with result code:", rc)

client = mqtt.Client()
client.username_pw_set(USERNAME, PASSWORD)
client.on_connect = on_connect
client.connect(BROKER, PORT, 60)
client.loop_start()

while True:
    # Generate dummy values
    rpm  = 800 + random.randint(-30, 30)
    ph   = 7.0 + random.uniform(-0.1, 0.1)
    temp = 37.0 + random.uniform(-0.5, 0.5)

    # Publish sensor data
    client.publish(TOPIC_RPM_STATE,  rpm)
    client.publish(TOPIC_PH_STATE,   ph)
    client.publish(TOPIC_TEMP_STATE, temp)

    # Publish constant setpoints 
    client.publish(TOPIC_RPM_SETPOINT,  SETPOINT_RPM)
    client.publish(TOPIC_PH_SETPOINT,   SETPOINT_PH)
    client.publish(TOPIC_TEMP_SETPOINT, SETPOINT_TEMP)

    print(f"Published → RPM:{rpm}  pH:{ph:.2f}  T:{temp:.2f}")

    time.sleep(1)

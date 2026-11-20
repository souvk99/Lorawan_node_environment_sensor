import paho.mqtt.client as mqtt
import time
import json

def on_connect(client, userdata, flags, rc, properties=None):
    print("MQTT Connected (rc=%s). Subscribing to application/# ..." % rc)
    client.subscribe("application/#")
    client.subscribe("+/gateway/#")

def on_message(client, userdata, msg):
    print(f"\n[RECEIVED] Topic: {msg.topic}")
    try:
        data = json.loads(msg.payload.decode('utf-8'))
        print("Decoded JSON keys:", list(data.keys()))
        if "deviceInfo" in data:
            print("DeviceInfo:", data["deviceInfo"])
        if "data" in data:
            print("Base64 Data:", data["data"])
        if "fPort" in data:
            print("fPort:", data["fPort"])
    except Exception:
        print("Raw bytes:", msg.payload[:80])

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message
client.connect("172.16.50.30", 1883, 60)
client.loop_start()

print("Listening for 70 seconds...")
time.sleep(70)
client.loop_stop()
print("Done.")

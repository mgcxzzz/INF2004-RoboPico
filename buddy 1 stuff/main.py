import network
import time
from umqtt.simple import MQTTClient

SSID = "ur wifi ssid"
PASSWORD = "ur wifi pw"

MQTT_BROKER = "ur ip"
MQTT_TOPIC = b"car/telemetry"

# Connect to Wi-Fi
wlan = network.WLAN(network.STA_IF)
wlan.active(True)

print("Connecting to WiFi...")

if not wlan.isconnected():
    wlan.connect(SSID, PASSWORD)

while not wlan.isconnected():
    print(".", end="")
    time.sleep(1)

print()
print("WiFi connected!")
print("IP address:", wlan.ifconfig()[0])

# Connect to MQTT and publish heartbeat
while True:
    client = None

    try:
        client = MQTTClient(
            "buddy1-pico-w",
            MQTT_BROKER,
            port=1883,
            keepalive=60
        )
        client.connect()
        print("MQTT connected!")

        while True:
            client.publish(
                MQTT_TOPIC,
                b"Buddy1 heartbeat"
            )
            print("Heartbeat sent")
            time.sleep(1)

    except Exception as e:
        print("MQTT error:", e)

        if client is not None:
            try:
                client.disconnect()
            except Exception:
                pass

        time.sleep(2)

        # Reconnect Wi-Fi if it was lost
        if not wlan.isconnected():
            wlan.connect(SSID, PASSWORD)
            while not wlan.isconnected():
                time.sleep(1)

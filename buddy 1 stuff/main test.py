
import network
import time
from umqtt.simple import MQTTClient

SSID = "ur wifi ssid"
PASSWORD = "ur wifi pw"
MQTT_BROKER = "ur ip"
MQTT_TOPIC = b"car/telemetry"

wlan = network.WLAN(network.STA_IF)
wlan.active(True)

if not wlan.isconnected():
    wlan.connect(SSID, PASSWORD)
    while not wlan.isconnected():
        time.sleep(1)

print("WiFi connected:", wlan.ifconfig()[0])

sequence = 0

while True:
    client = None
    try:
        client = MQTTClient(
            "buddy1-delivery-test",
            MQTT_BROKER,
            port=1883,
            keepalive=60
        )
        client.connect()
        print("MQTT connected!")

        while True:
            sequence += 1
            message = "heartbeat:{}".format(sequence)
            client.publish(MQTT_TOPIC, message.encode())
            print("Sent:", sequence)
            time.sleep(1)

    except Exception as e:
        print("MQTT error:", e)
        if client:
            try:
                client.disconnect()
            except Exception:
                pass
        time.sleep(2)


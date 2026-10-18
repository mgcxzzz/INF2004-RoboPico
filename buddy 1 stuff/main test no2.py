
import network
import time
from umqtt.simple import MQTTClient

SSID = "ur wifi ssid"
PASSWORD = "ur wifi pw"
MQTT_BROKER = "ur ip"

COMMAND_TOPIC = b"car/commands"
SAFETY_TOPIC = b"car/safety"
TELEMETRY_TOPIC = b"car/telemetry"

ALLOWED_COMMANDS = (
    b"start",
    b"pause",
    b"resume",
    b"stop",
    b"reset",
)

def handle_command(topic, message):
    command = message.strip().lower()

    if command in ALLOWED_COMMANDS:
        print("Accepted command:", command)

    elif command == b"emergency_stop":
        print("Emergency stop received!")
        client.publish(SAFETY_TOPIC, b"emergency_stop")
        print("Safety notification published")

    else:
        print("REJECTED command:", command)

wlan = network.WLAN(network.STA_IF)
wlan.active(True)

if not wlan.isconnected():
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    while not wlan.isconnected():
        time.sleep(1)

print("WiFi connected:", wlan.ifconfig()[0])

while True:
    client = None

    try:
        client = MQTTClient(
            "buddy1-command-test",
            MQTT_BROKER,
            port=1883,
            keepalive=60
        )
        client.set_callback(handle_command)
        client.connect()
        client.subscribe(COMMAND_TOPIC)

        print("MQTT connected")
        print("Listening for commands...")

        last_heartbeat = time.ticks_ms()

        while True:
            client.check_msg()

            if time.ticks_diff(
                time.ticks_ms(), last_heartbeat
            ) >= 1000:
                client.publish(
                    TELEMETRY_TOPIC,
                    b"Buddy1 heartbeat"
                )
                print("Heartbeat sent")
                last_heartbeat = time.ticks_ms()

            time.sleep_ms(20)

    except Exception as e:
        print("Connection lost:", e)

        if client is not None:
            try:
                client.disconnect()
            except Exception:
                pass

        time.sleep(2)

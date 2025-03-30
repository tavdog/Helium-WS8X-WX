import threading
import time
import os
import sys
import json
import paho.mqtt.client as mqtt
from Adafruit_IO import Client, Feed# , Data, RequestError
import pytz
import subprocess
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()


class WatchdogTimer:
    def __init__(self, timeout, user_handler=None):
        self.timeout = timeout
        self.handler = user_handler if user_handler else self.default_handler
        self.timer = threading.Timer(self.timeout, self.handler)

    def reset(self):
        self.timer.cancel()
        self.timer = threading.Timer(self.timeout, self.handler)
        self.timer.start()

    def stop(self):
        self.timer.cancel()

    def default_handler(self):
        print("Watchdog timer expired! Exiting script...")
        sys.exit(1)


# Example usage
def watchdog_handler():
    print("Watchdog timer expired! Exiting script...")
    os._exit(1)


watchdog = WatchdogTimer(900, watchdog_handler)  # seconds 300 = 5 min

# Start the watchdog timer
watchdog.reset()

# MQTT
BROKER = os.getenv("MQTT_BROKER")
USER = os.getenv("MQTT_USER")
PASS = os.getenv("MQTT_PASS")
PORT = int(os.getenv("MQTT_PORT", 1884))
TOPICS = os.getenv("MQTT_TOPICS", "").split(",")

# AIO
AIO_USER = os.getenv("AIO_USER")
AIO_KEY = os.getenv("AIO_KEY")
AIO_FEED_GROUP = os.getenv("AIO_FEED_GROUP")
TIMEZONE = os.getenv("TIMEZONE", "US/Hawaii")
aio = Client(AIO_USER, AIO_KEY)


my_timezone = pytz.timezone(TIMEZONE)


# get feed object or create it if it doesn't exist yet.
def get_feed(name, kind):
    # name = full_name.split("-")[0].lower()
    name = name.lower()
    kind = kind.lower()
    kind = kind.replace(" ","").replace("speed","avg").replace("direction","dir")
    print("getting feed: " + name + "-" + kind)
    feed = None
    try:
        feed = aio.feeds(f"{AIO_FEED_GROUP}.{name}-{kind}")
        print(f"key is {feed.key}")
    except Exception as e:
        if True:
            print("creating feed:" + f"{AIO_FEED_GROUP}.{name}-{kind}")
            # Create Feed
            new_feed = Feed(name=f"{name}_{kind}")
            feed = aio.create_feed(feed=new_feed, group_key=AIO_FEED_GROUP)

        else:
            print("Unknown Feed, not creating")
            return None

    return feed


def degrees_to_cardinal(degrees):
    directions = [
        "N",
        "NNE",
        "NE",
        "ENE",
        "E",
        "ESE",
        "SE",
        "SSE",
        "S",
        "SSW",
        "SW",
        "WSW",
        "W",
        "WNW",
        "NW",
        "NNW",
        "N",
    ]
    index = round(degrees / 22.5) % 16
    return directions[index]


# Callback when the client connects to the broker
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT broker!")
        for topic in TOPICS:
            client.subscribe(topic)
            print(f"Subscribed to topic: {topic}")
    else:
        print(f"Failed to connect, return code {rc}")


# Callback when a message is received
def on_message(client, userdata, msg):
    watchdog.reset()
    try:
        # Clean and parse the payload
        raw_msg = msg.payload.decode('utf-8')
        
        # If the payload is already a dictionary (string representation)
        if raw_msg.startswith("{") and raw_msg.endswith("}"):
            try:
                # Convert string representation of dict to actual dict
                payload = eval(raw_msg)
            except:
                # If eval fails, try standard JSON parsing
                payload = json.loads(raw_msg)
        else:
            payload = json.loads(raw_msg)

        # print(f"got payload : {payload}")
        
        # Extract device name and data
        device_name = payload.get("name", "")
        if "decoded" in payload and "payload" in payload["decoded"]:
            data = payload["decoded"]["payload"]
            
            wind_data = {
                "wind_speed": data.get("velAvg", 0),
                "wind_gust": data.get("gust", 0),
                "wind_lull": data.get("lull", 0),
                "wind_direction": data.get("dir", 0),
                "temperature": data.get("tempC", None),
                "batV": data.get("batV", None),
                "node_mv": data.get("node_mv", None),
            }
            
            # Publish to Adafruit IO
            # publish_wind(device_name, wind_data)
            publish_node_mv(device_name, wind_data)

    except json.JSONDecodeError as e:
        print(f"Error decoding JSON: {e}")
        print(f"Raw message: {msg.payload.decode('utf-8')}")
    except Exception as e:
        print(f"Error processing message: {e}")
        print(f"Raw message: {msg.payload}")


def mph(ms):
    return round(ms*2.237,1) if ms else None


def publish_node_mv(node_name, data):
    print(f"publishing mv from {node_name}")

    # Publish to Adafruit IO
    field = "nodemv"
    val = data.get("node_mv",0)
    feed = get_feed(node_name,field)
    key = feed.key #f"{AIO_FEED_GROUP}.{node_name.lower()}-{field}"
    print(f"aio send {key} {val}")
    aio.send_data(key, val, None)

    field = "batV"
    val = data.get("batV", 0)
    feed = get_feed(node_name,field)
    key = feed.key #f"{AIO_FEED_GROUP}.{node_name.lower()}-{field}"
    print(f"aio send {key} {val}")
    aio.send_data(key, val, None)


def publish_wind(node_name, data):
    if node_name.startswith("WS8"):  # Adjust this condition based on your device naming
        print(f"publishing wind from {node_name}")

        # Convert m/s to mph if needed
        data["wind_gust"] = mph(data["wind_gust"])
        data["wind_speed"] = mph(data["wind_speed"])
        data["wind_lull"] = mph(data["wind_lull"])

        # Apply Hookipa correction if needed
        if not (130 < data["wind_direction"] < 300) and "ws8a" in node_name.lower():
            print("applying 15% correction for hookipa")
            data["wind_gust"] *= 0.85
            data["wind_speed"] *= 0.85
            data["wind_lull"] *= 0.85
            data["wind_gust"] = (data["wind_gust"] * 2 + data["wind_speed"]) / 3

        # Publish to Adafruit IO
        for field, val in data.items():
            try:
                field = (
                    field.replace("wind_", "")
                    .replace("speed", "avg")
                    .replace("direction", "dir")
                    .replace("temperature", "tempc")
                )

                key = f"{AIO_FEED_GROUP}.{node_name.lower()}-{field}"
                print(f"aio send {key} {val}")
                aio.send_data(key, val, None)
            except Exception as e:
                print(f"problem sending feed with key {key} {e}")

        # Generate CSV format
        cardinal = degrees_to_cardinal(data["wind_direction"])
        if data["temperature"] is not None:
            csv = f"{data['wind_speed']},{data['wind_gust']},{data['wind_lull']},{cardinal},{data['wind_direction']},{round(data['temperature'],1)}"
        else:
            csv = f"{data['wind_speed']},{data['wind_gust']},{data['wind_lull']},{cardinal},{data['wind_direction']}"

        # Send CSV data
        aio.send_data(f"{AIO_FEED_GROUP}.{node_name.lower()}-csv", csv, None)

        # Write to files
        output_file = f"/home/taviz/wildc.net/wind/{node_name}.csv"
        with open(output_file, mode="w") as file:
            file.write(csv)

        output_file = f"/home/taviz/wildc.net/wind/{node_name}"
        with open(output_file, mode="w") as file:
            file.write(
                f"{cardinal} {data['wind_direction']} {round(data['wind_speed'])}g{round(data['wind_gust'])}"
            )

        # Handle Windguru updates
        if node_name == "WS8A":
            print("sending WS8A to windguru")
            try:
                subprocess.run(
                    "cd ~/wildc.net/wind && python3 hookipa_station_to_windguru.py >> windguru.log 2>&1",
                    shell=True,
                )
            except Exception as e:
                print(f"windguru send failed {e}")
        print("---------")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set(USER, PASS)
while(True):
    try:
        client.connect(BROKER, PORT, keepalive=60)
        client.loop_forever()
    except Exception as e:
        print(f"An error occurred: {e}")

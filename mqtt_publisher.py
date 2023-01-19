# publish to the mqtt topic SistCiberFis_jacinto/SCF/Temp the message "Hello World" every 5 seconds
import paho.mqtt.client as mqtt
import time

client = mqtt.Client()
client.connect("test.mosquitto.org", 1883, 60)

while True:
    client.publish("SistCiberFis_jacinto/SCF/Temp", 23.45)
    time.sleep(5)
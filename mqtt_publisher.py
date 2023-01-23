# publish to the mqtt topic SCF/trabajoFinal/Humdidity the message "Hello World" every 5 seconds
import paho.mqtt.client as mqtt
import time

client = mqtt.Client()
client.connect("test.mosquitto.org", 1883, 60)

i = 0;
while True:
    client.publish("SCF/trabajoFinal/TempSup", str(i))
    i = i + 1
    time.sleep(5)
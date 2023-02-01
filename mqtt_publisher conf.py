# publish to the mqtt topic SCF/trabajoFinal/Humdidity the message "Hello World" every 5 seconds
import paho.mqtt.client as mqtt
import time
import random

client = mqtt.Client()
client.connect("test.mosquitto.org", 1883, 60)

i = 0;


# Esta linea para cambiar la hora
client.publish("SCF/trabajoFinal/conf/RTC", "20:20:20,20/10/20")

# Esta linea para cambiar el acelerometro
client.publish("SCF/trabajoFinal/conf/Accel", "104,4")

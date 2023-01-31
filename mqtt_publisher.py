# publish to the mqtt topic SCF/trabajoFinal/Humdidity the message "Hello World" every 5 seconds
import paho.mqtt.client as mqtt
import time
import random

client = mqtt.Client()
client.connect("test.mosquitto.org", 1883, 60)

i = 0;
client.publish("SCF/trabajoFinal/conf", "20:20:20,20/10/20")
# while True:
#     #client.publish("SCF/trabajoFinal/TempSup", str(i))
#     # i = random number 1 or 0
#     i = random.randint(0, 1)



#     #i = 1
#     print("Mensaje publicado: ", str(i))
#     time.sleep(15)
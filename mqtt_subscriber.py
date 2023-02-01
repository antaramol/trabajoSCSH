# subscribe to the mqtt topic SCF/trabajoFinal/Humdidity and print the message
import paho.mqtt.client as mqtt

# delete file data.txt if it exists
import os
if os.path.exists('accel_received.txt'):
    os.remove('accel_received.txt')

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("SCF/trabajoFinal/Accel")

def on_message(client, userdata, msg):
    # append a line to the file with the msg.payload
    with open('accel_received.txt', 'a') as f:
        f.write(msg.payload.decode("utf-8") + '\n')
    print(msg.topic+" "+str(msg.payload))

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("test.mosquitto.org", 1883, 60)

client.loop_forever()



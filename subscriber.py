#!/usr/bin/env python3

import paho.mqtt.client as mqtt

from ev3dev.ev3 import *

# This is the Subscriber

def on_connect(client, userdata, flags,message):

    client.subscribe("topic/sensors")
    print('conectado')

def on_message(client, userdata, message):
    carga = message.payload.decode()
    print("carga:", carga)
    client.disconnet()


client = mqtt.Client()
client.connect("10.42.0.243", 1883, 60)
print('Conectado.')

client.on_connect = on_connect
client.on_message = on_message

client.loop_start()

while True:
    pass

#client.loop_forever()
#print('Em loop')
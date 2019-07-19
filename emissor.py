#!/usr/bin/env python3

from ev3dev.ev3 import *


import paho.mqtt.client as mqtt
import struct

sensor_obs = ColorSensor('in1')
infra = InfraredSensor()
sensor_obs.mode = 'COL-COLOR'
infra.mode_ir_prox = 'IR-PROX'
client = mqtt.Client()
client.connect("localhost",1883,60)

client.loop_start()

lista = [1,2]
tupla = (1,2)
try:
    while True:

        valor =sensor_obs.value()
        valor2 =infra.value()

        c=struct.pack('ii',valor,valor2)
        client.publish("topic/teste",c)
        print(struct.unpack('ii',c))

        time.sleep(0.1)

except KeyboardInterrupt:
    pass

client.loop_end()
client.disconnect();

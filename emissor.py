#!/usr/bin/env python3

from ev3dev.ev3 import *


import paho.mqtt.client as mqtt
import struct

sensor_obs = ColorSensor('in1')

infra = InfraredSensor()

sensor_cor = ColorSensor('in3')
sensor_cor2 = ColorSensor('in4')

sensor_obs.mode = 'COL-COLOR'

sensor_cor.mode = 'COL-COLOR'
sensor_cor2.mode ='COL-COLOR'
infra.mode = 'IR-PROX'


client = mqtt.Client()
client.connect("localhost",1883,60)

client.loop_start()

try:
    while True:

        valor =sensor_obs.value()
        valor2 =infra.value()
        valor3 = sensor_cor.value()
        valor4 = sensor_cor2.value()
        c=struct.pack('iiii',valor,valor2,valor3,valor4)
        client.publish("topic/teste",c)
        print(struct.unpack('iiii',c))

        time.sleep(0.1)

except KeyboardInterrupt:
    pass

client.loop_end()
client.disconnect();

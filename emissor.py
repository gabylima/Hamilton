#!/usr/bin/env python3

from ev3dev.ev3 import *


import paho.mqtt.client as mqtt

sensor_obs = ColorSensor('in1')
sensor_obs.mode = 'COL-COLOR'

client = mqtt.Client()
client.connect("localhost",1883,60)

client.loop_start()


try:
    while True:
        valor =sensor_obs.value()
        if(valor== 1 or valor ==0):
            #client.publish("topic/test", str());

            #message = pack("sensor obstaculo",valor, time.time())

            client.publish("topic/sensors", str(valor));
            print(valor)
            time.sleep(0.1)

        else:
            print(valor)

except KeyboardInterrupt:
    pass

client.loop_end()
client.disconnect();


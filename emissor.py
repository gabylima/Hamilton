#!/usr/bin/env python3

from ev3dev.ev3 import *


import paho.mqtt.client as mqtt

sensor_obs = ColorSensor('in1')
infra = InfraredSensor()
sensor_obs.mode = 'COL-COLOR'
infra.mode_ir_prox = 'IR-PROX'
client = mqtt.Client()
client.connect("localhost",1883,60)

client.loop_start()



try:
    while True:
        valor =sensor_obs.value()
        valor2 =infra.value()

        if(valor== 1):
            #client.publish("topic/test", str());
            #message = pack("sensor obstaculo",valor, time.time())

            client.publish("topic/sensors", str(valor))
            print('sensor_cor',valor)
            time.sleep(0.1)

        elif (infra.value() <=4):
            client.publish("topic/infra", str(valor2))
            print('infra:',valor2)
            time.sleep(0.1)

        else:
            print('i:', infra.value())
            print(valor)

except KeyboardInterrupt:
    pass

client.loop_end()
client.disconnect();

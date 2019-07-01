#!/usr/bin/env python3

from ev3dev.ev3 import *

import paho.mqtt.client as mqtt

client = mqtt.Client()
client.connect("localhost",1883,60)
client.publish("topic/test", "Hello world!");
client.disconnect();
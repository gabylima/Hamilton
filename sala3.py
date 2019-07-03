#!/usr/bin/env python3

from ev3dev.ev3 import *
from time import sleep
from os import system
from simple_pid import PID
import paho.mqtt.client as mqtt
from csv import*

TPSUB=2000
TPDES=400
# Motores

garra =MediumMotor('outC')


def descer ():

    garra.run_to_abs_pos(speed_sp=TPDES)
    garra.run_to_abs_pos(speed_sp=TPDES)
    garra.run_to_abs_pos(speed_sp=TPDES)


def subir ():

    garra.run_to_abs_pos(speed_sp=-TPSUB)
    garra.run_to_abs_pos(speed_sp=-TPSUB)
    garra.run_to_abs_pos(speed_sp=-TPSUB)


#descer()
#sleep(0.5)

subir()




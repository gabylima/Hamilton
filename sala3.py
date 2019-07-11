#!/usr/bin/env python3

from ev3dev.ev3 import *
from time import sleep
from os import system
from simple_pid import PID
import paho.mqtt.client as mqtt
from csv import*

TPSUB=300
TPDES=900
system('setfont Lat15-TerminusBold14') # estilização
KP = 11 #modifiquei o kp: tava 12
KI = 0
KD = 0
TP = 210.0 #modifiquei tp: tava 280
VALOR_MAX_CONTROL = 1000 - TP
CORRECAO_MOTOR = 10
carga =-1



# Motores
motor_dir = LargeMotor('outB')
motor_esq = LargeMotor('outD')
garra =MediumMotor('outA')

#sensores
sensor_esq = ColorSensor("in1")
sensor_dir = ColorSensor("in2")

sensor_esq.mode = 'COL-REFLECT'
sensor_dir.mode = 'COL-REFLECT'

# Sensor ultrassonico
ultra1 = UltrasonicSensor('in3')
ultra2= UltrasonicSensor('in4')


garra =MediumMotor('outA')
TPDES=900

def stop():
    # O robô para de se mover com os dois motores ao mesmo tempo

    motor_esq.stop()
    motor_dir.stop()
    sleep(0.5)

def descer ():
    garra.run_timed(time_sp=1500, speed_sp=-TPDES)
def subir ():

    garra.run_to_abs_pos(speed_sp=-TPSUB)
    garra.run_to_abs_pos(speed_sp=-TPSUB)
    garra.run_to_abs_pos(speed_sp=-TPSUB)


descer()



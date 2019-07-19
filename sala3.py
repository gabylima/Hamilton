#!/usr/bin/env python3

from ev3dev.ev3 import *
from time import sleep
from os import system
from simple_pid import PID
import paho.mqtt.client as mqtt
from csv import*


system('setfont Lat15-TerminusBold14') # estilização
KP = 11 #modifiquei o kp: tava 12
KI = 0
KD = 0
TP = 210.0 #modifiquei tp: tava 280
VALOR_MAX_CONTROL = 1000 - TP
CORRECAO_MOTOR = 10
carga =-1
TPDES=10
TPDES1=300

TPSUB=50
TPSUB1=80


# Motores
#motor_dir = LargeMotor('outB')
#motor_esq = LargeMotor('outD')
garra = LargeMotor('outA')
garra2 = MediumMotor('outD')

#sensores
#sensor_esq = ColorSensor("in1")
#sensor_dir = ColorSensor("in2")

#sensor_esq.mode = 'COL-REFLECT'
#sensor_dir.mode = 'COL-REFLECT'

# Sensor ultrassonico
#ultra1 = UltrasonicSensor('in3')
#ultra2= UltrasonicSensor('in4')



def descer ():
    garra.run_timed(time_sp=1500, speed_sp=TPDES1)
    sleep(0.9)
    garra2.run_timed(time_sp=1500, speed_sp=TPDES)
def subir ():
    garra2.run_timed(time_sp=1500, speed_sp=-TPSUB)
    sleep(0.9)
    garra.run_timed(time_sp=1500, speed_sp=-TPSUB1)
    sleep(0.9)
    garra2.run_timed(time_sp=1500, speed_sp=50)

#def frente():
    #motor_dir.run_to_rel_pos(position_sp=140, speed_sp=400, stop_action="hold")
    #motor_esq.run_to_rel_pos(position_sp=140, speed_sp=400, stop_action="hold")
    #sleep(0.5)

#descer()

subir()



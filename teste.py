#!/usr/bin/env python3

from ev3dev.ev3 import *
from time import sleep
from os import system
from simple_pid import PID
import paho.mqtt.client as mqtt
from datetime import datetime, timedelta
import  struct
from csv import *

system('setfont Lat20-TerminusBold14') # estilização
KP = 10 #modifiquei o kp: tava 12
KI = 0
KD = 0
TP = 250.0 #modifiquei tp: tava 280
VALOR_MAX_CONTROL = 1000 - TP
CORRECAO_MOTOR = 0
carga =" "
TPDES=900
cont_rampa = 0
sensor_cor=0
TPSUB=900
TPS3 = 900
TP_procurar = 400
cont_s3 = -1
contador_lado = 0
contLateral = 0
# Motores
motor_dir = LargeMotor('outD')
motor_esq = LargeMotor('outB')
#garra =MediumMotor('outA')
garra2 = MediumMotor('outA')


#sensores
sensor_esq = ColorSensor("in1")
sensor_dir = ColorSensor("in2")

sensor_esq.mode = 'COL-REFLECT'
sensor_dir.mode = 'COL-REFLECT'

# Sensor ultrassonico
ultra1 = UltrasonicSensor('in3')
ultra2= UltrasonicSensor('in4')

# Modo Color. * 0=desconhecido, 1=preto, 2=azul, 3=verde, 4=amarelo, 5=vermelho, 6=branco, 7=marrom

def stop():
    # O robô para de se mover com os dois motores ao mesmo tempo
    motor_esq.stop()
    motor_dir.stop()
    sleep(0.5)

pid = PID(KP, KI, KD, setpoint=0)
while True:
    diferenca = sensor_esq.value() - sensor_dir.value()
    control = pid(diferenca)
    if (diferenca == 0):
        stop()
        break
    print(diferenca)
    motor_esq.run_forever(speed_sp = -control)
    motor_dir.run_forever(speed_sp = control)
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
carga =" "
#TPDES=100
TPDES=300
TPDES1=300

#TPSUB=100
TPSUB=900
TPSUB1=80


# Motores
motor_dir = LargeMotor('outB')
motor_esq = LargeMotor('outD')
#garra = LargeMotor('outA')
#garra2 = MediumMotor('outC')

garra2 = MediumMotor('outA')

#sensores
ultra1 = UltrasonicSensor('in3')
ultra2= UltrasonicSensor('in4')


def subirr():
    garra2.run_timed(time_sp=1500, speed_sp=TPSUB)
    sleep(0.9)


def descerr():

    garra2.run_timed(time_sp=1500, speed_sp=-TPDES)
    sleep(0.9)

def mprocurar():
    client = mqtt.Client()
    Conectar(client)
    sleep(0.5)
    subirr()
    sleep(0.5)
    giroD()
    sleep(1.5)
    try:
        while True:
            valor = ultra1.value()
            if(valor<=55):
                re()
                re()
                vertiE()
                sleep(0.5)
                stop()
                if(carga[1]>=18 ) :
                   frente()
                   frente()
                   frente()
                   sleep(0.5)
                   descerr()
                   sleep(0.5)
                   vertiE()
                   sleep(0.5)
                   mre()
                   sleep(0.5)
                   frente()
                   sleep(0.5)
                   mre()
                   sleep(0.5)
                   frente()
                   sleep(0.5)
                   mre()
                   sleep(0.5)
                   frente()
                   stop()
                   break
                else:
                    if(valor <= 80 and carga[1] <= 20):
                        stop()
                        re()
                    elif(valor <= 80 and carga[1]>=40):
                        stop()
                        re()

            motor_dir.run_forever(speed_sp=TP)
            motor_esq.run_forever(speed_sp=TP)
    except KeyboardInterrupt:
            stop()

def on_connect(client, userdata, flags, message):
    client.subscribe("topic/teste")

def on_message(client, userdata, message):
    global carga
    carga = unpack('iiii', message.payload)

def Conectar(client):
    client.connect("10.42.0.183", 1883, 60)
    client.on_connect = on_connect
    client.on_message = on_message
    client.loop_start()

def Desconectar(client):
    client.loop_stop()
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




def descer ():
    garra.run_timed(time_sp=1500, speed_sp=TPDES1)
    sleep(0.9)
    garra2.run_timed(time_sp=1500, speed_sp=TPDES)
    sleep(0.9)

def Egiro():
    #gira para a esquerda
    motor_esq.run_to_rel_pos(position_sp=-540, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=540, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=-180, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=180, speed_sp=400, stop_action="hold")
    sleep(0.5)

def Dgiro():

    #gira para a direita
    motor_esq.run_to_rel_pos(position_sp=540, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-540, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=180, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-180, speed_sp=400, stop_action="hold")
    sleep(0.5)

def subir ():
    garra2.run_timed(time_sp=1500, speed_sp=-TPSUB)
    sleep(0.9)
    garra.run_timed(time_sp=1500, speed_sp=-TPSUB1)
    sleep(0.9)
    garra2.run_timed(time_sp=1500, speed_sp=40)

#def frente():
    #motor_dir.run_to_rel_pos(position_sp=140, speed_sp=400, stop_action="hold")
    #motor_esq.run_to_rel_pos(position_sp=140, speed_sp=400, stop_action="hold")
    #sleep(0.5)

#descer()

def movimento():
    motor_esq.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=300, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=300, speed_sp=400, stop_action="hold")
    sleep(0.5)
'''
descer()
sleep(0.5)
movimento()
sleep(0.5)
subir()

'''
def stop():
    motor_dir.stop()
    motor_esq.stop()

def frente():

    motor_esq.run_to_rel_pos(position_sp=100, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=100, speed_sp=400, stop_action="hold")
    sleep(0.3)




def mdescer():
    garra.run_timed(time_sp=1500, speed_sp=TPDES1)
    sleep(0.9)

def mover():

    try:
        while True:
            valor = ultra1.value()
            if (valor >= 100 and valor <=190):
                stop()
                mdescer()
                sleep(0.5)
                subir()
                sleep(0.5)
                stop()
            if(valor <= 60):
                stop()
                Egiro()

                frente()
                Egiro()
            motor_dir.run_forever(speed_sp=TP)
            motor_esq.run_forever(speed_sp=TP)
    except KeyboardInterrupt:
        motor_esq.stop()
        motor_dir.stop()

def manual():

    try:
        while True:
            valor = ultra1.value()

            frente()

            if (valor >= 100 and valor <= 190 ):
                stop()
                mdescer()
                sleep(0.5)
                subir()
                sleep(0.5)
                stop()

            if (valor <= 60):
                stop()
                Egiro()

                frente()
                Egiro()


    except KeyboardInterrupt:
        motor_esq.stop()
        motor_dir.stop()


#mover()
#manual()

#subirr()
#sleep(0.5)
#subir()

def descerr():

    garra2.run_timed(time_sp=1500, speed_sp=-TPDES)
    sleep(0.9)

def frentee():
    for i in range(0, 30):
        frente()

    stop()

def re ():
    motor_esq.run_to_rel_pos(position_sp=-60, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-60, speed_sp=400, stop_action="hold")
    sleep(0.5)

def reFinal():
    motor_esq.run_to_rel_pos(position_sp=-200, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-200, speed_sp=400, stop_action="hold")
    sleep(0.5)



def giroE():

    motor_esq.run_to_rel_pos(position_sp=-540, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=540, speed_sp=400, stop_action="hold")
    sleep(0.5)

def frentinha():
    motor_esq.run_to_rel_pos(position_sp=500, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=500, speed_sp=400, stop_action="hold")
    sleep(0.5)

def giroD():
    motor_esq.run_to_rel_pos(position_sp=540, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-540, speed_sp=400, stop_action="hold")
    sleep(0.5)
def sala3 ():

    try:

        descerr()

        #1º etapa

        frentee()
        re()
        giroE()
        sleep(1.5)
        frentinha()
        sleep(2.0)
        giroE()
        sleep(1.5)

        #2ºetapa, depois que girar totalmente

        frentee()
        sleep(1.5)
        re()
        giroD()
        sleep(1.5)
        frentinha()
        sleep(2.0)
        giroD()
        sleep(1.5)

        #3º etapa, depois de girar totalmente

        frentee()
        re()
        giroE()
        sleep(1.5)
        frentinha()
        sleep(2.0)
        giroE()
        sleep(1.5)

        #4º etapa, depois de girar totalmente

        frentee()
        re()
        giroD()
        sleep(1.5)
        frentinha()
        sleep(2.0)
        giroD()
        sleep(1.5)

        #5º etapa, depois de girar totalmente

        frentee()

        #etapa final
        reFinal()
        sleep(0.5)
        subirr()




    except KeyboardInterrupt:
        motor_esq.stop()
        motor_dir.stop()

sala3()
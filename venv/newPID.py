#!/usr/bin/env python3

from ev3dev.ev3 import *
from time import sleep
from os import system
from simple_pid import PID


KP = 10
KI = 0
KD = 0
TP = 150.0

#di branco = 79
#di negro = 7

#esq branco =81
#esq negro =6



system('setfont Lat15-TerminusBold14')

# Motores
motor_esq = LargeMotor('outB')
motor_dir = LargeMotor('outD')

sensor_esq = ColorSensor("in1")
sensor_dir = ColorSensor("in2")

# Put the color sensor into RGB mode.
sensor_esq.mode = 'COL-REFLECT'
sensor_dir.mode = 'COL-REFLECT'



def calibragem():
    botao = Button()
    print('posicione os sensores de cor na superficie branca e aperte um botao')
    try:
        while True:
            if botao.any():
                   di = sensor_dir.value()
                   esq = sensor_esq.value()
                   return esq - di
            else:
                sleep(0.01)

    except KeyboardInterrupt:
        motor_dir.stop()
        motor_esq.stop()

def executar(TP, SP):
    pid = PID(KP, KI, KD, setpoint=SP)
    try:
        while True:

            dif = sensor_esq.value() - sensor_dir.value()
            control = pid(dif)

            if (control > 700):
                control = 700
            elif (control < -700) :
                control= -700

            print('dif:' + str(dif))
            print(control)

            # offset = 5  # margem de erro para que ele fique reto na linha
            # erro = ( + offset)  # Calcula o erro para que ele sempre siga a linha preta
            # p = kp * erro  # constante proporcionalss
            # # anda de acordo com o erro calculado

            motor_esq.run_forever(speed_sp=TP + control)
            motor_dir.run_forever(speed_sp=TP - control)

    except KeyboardInterrupt:
        motor_dir.stop()
        motor_esq.stop()

SP = calibragem()
executar(TP,SP)

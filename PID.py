#!/usr/bin/env python3

from ev3dev.ev3 import *
from time import sleep
from os import system
from simple_pid import PID

KP = 7
KI = 0
KD = 0
SP = -6
TP = -180.0

pid = PID(KP, KI, KD, setpoint=SP)


system('setfont Lat15-TerminusBold14')

# Motores
motor_esq = LargeMotor('outA')
motor_dir = LargeMotor('outD')

# Connect EV3 color and touch sensors to any sensor ports
sensor_esq = ColorSensor("in1")
sensor_dir = ColorSensor("in3")

# Put the color sensor into RGB mode.
sensor_esq.mode = 'COL-REFLECT'
sensor_dir.mode = 'COL-REFLECT'

def executar(TP):
    try:
        while True:
            control = pid((sensor_esq.value() - sensor_dir.value()))

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


executar(TP)



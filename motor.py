#!/usr/bin/env python3

from ev3dev.ev3 import *
from time import sleep
from os import system
from simple_pid import PID
import paho.mqtt.client as mqtt

system('setfont Lat15-TerminusBold14') # estilização





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

def calibragem(button2):
    print('posicione os sensores de cor na superficie branca e aperte um botao')
    try:
        while True:
            if button2.enter:

                   di = sensor_dir.value()
                   esq = sensor_esq.value()
                   return esq - di
            else:
                sleep(0.01)

    except KeyboardInterrupt:
        motor_dir.stop()
        motor_esq.stop()

def stop():
    # O robô para de se mover com os dois motores ao mesmo tempo

    motor_esq.stop()
    motor_dir.stop()
    sleep(0.5)

def motor():

        button = Button()
        stop()
        KPO = 11
        TPO = 0
        KIO = 0
        KDO = 0
        SPO =calibragem(button)
        print(SPO)
        V_MAX_MOTOR = 1000
        VALOR_MAX_CONTROL = V_MAX_MOTOR - TPO

        pid = PID(KPO, KIO, KDO, setpoint=SPO)
        while True:
            try:
                dif = sensor_esq.value() - sensor_dir.value()
                control = pid(dif)
                if (control > VALOR_MAX_CONTROL):
                    control = VALOR_MAX_CONTROL
                elif (control < -VALOR_MAX_CONTROL):
                    control = -VALOR_MAX_CONTROL
                motor_esq.run_forever(speed_sp=TPO)
                motor_dir.run_forever(speed_sp=TPO + control)
            except KeyboardInterrupt:
                motor_dir.stop()
                motor_esq.stop()
                break

motor()
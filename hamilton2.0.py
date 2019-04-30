#!/usr/bin/env python3

from ev3dev.ev3 import *
from time import sleep
from os import system
from simple_pid import PID
# Motores
motor_esq = LargeMotor('outD')
motor_dir = LargeMotor('outA')

x=400
pot=700


def tras():
    # Função para que o robô ande para trás por ângulo

    motor_dir.run_to_rel_pos(position_sp=-240, speed_sp=pot, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-240, speed_sp=pot, stop_action="hold")
    sleep(0.5)

def frente():
    # Função para que o robô ande para a frente por ângulo

    motor_dir.run_to_rel_pos(position_sp=240, speed_sp=pot, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=240, speed_sp=pot, stop_action="hold")
    sleep(0.5)

def teste():

    y=0
    try:

        while(y<3):

            tras()
            tras()
            frente()



            y=y+1
            print('p')
        motor_dir.stop()
        motor_esq.stop()
    except KeyboardInterrupt:
        motor_esq.stop()
        motor_dir.stop()


def moviment():

    #try:

        cont=0
        while True:

            if(cont<3):

                motor_esq.run_forever(speed_sp=x)
                motor_dir.run_forever(speed_sp=x)

                #motor_esq.run_to_rel_pos(position_sp=120, speed_sp=x, stop_action="hold")
                #motor_dir.run_to_rel_pos(position_sp=120, speed_sp=x, stop_action="hold")
                print("entrei no if")

                cont=cont+1
            else:

                motor_esq.stop()
                motor_dir.stop()
                print("entrei no else")
                break

    #except KeyboardInterrupt:
        #motor_esq.stop()
        #motor_dir.stop()


teste()

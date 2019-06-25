#!/usr/bin/env python3

from ev3dev.ev3 import *
from time import sleep
from os import system
from simple_pid import PID


motor_dir = LargeMotor('outB')
motor_esq = LargeMotor('outD')

TP = 280
while True:
    try:
        motor_esq.run_forever(speed_sp=TP )
        motor_dir.run_forever(speed_sp=TP+ 10)

    except KeyboardInterrupt:
        motor_dir.stop()
        motor_esq.stop()
        break
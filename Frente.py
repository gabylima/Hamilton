#!/usr/bin/env python3

from ev3dev.ev3 import *
from time import sleep
from os import system
from simple_pid import PID


motor_dir = LargeMotor('outB')
motor_esq = LargeMotor('outD')

TP = 280

try:
    #motor_esq.run_forever(speed_sp=TP )
    #motor_dir.run_forever(speed_sp=TP+ 10)

    motor_dir.run_to_rel_pos(position_sp=240, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=240, speed_sp=400, stop_action="hold")
    sleep(0.5)

except KeyboardInterrupt:
    motor_dir.stop()
    motor_esq.stop()

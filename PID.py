#!/usr/bin/env python3
from ev3dev.ev3 import *
motor_dir = LargeMotor('outB')
motor_esq = LargeMotor('outD')
TP = 210.0
for i in range (0,60):
    motor_dir.run_forever(speed_sp=TP)
    motor_esq.run_forever(speed_sp=TP)
motor_esq.stop()
motor_dir.stop()
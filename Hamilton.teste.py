#!/usr/bin/env python3

from ev3dev.ev3 import *
from time import sleep

motor_esq = LargeMotor('outA')
motor_dir = LargeMotor('outB')
sensor_esq=ColorSensor("in1")
sensor_dir=ColorSensor("in2")


def frente():
    motor_esq.run_timed(time_sp=2000, speed_sp= -500)
    motor_dir.run_timed(time_sp=2000, speed_sp= -500)
    sleep(2)

def lado():
    motor_esq.run_timed(time_sp=5700, speed_sp= 0)
    motor_dir.run_timed(time_sp=5700, speed_sp=180)
    sleep(2)

x=0
black=5
white=6

#while (x<=2):
   #lado()
   # x=x+1

sensor_esq.mode = 'COL-REFLECT'
sensor_dir.mode = 'COL-REFLECT'


while (x<1):
    s1=sensor_esq.value()
    s2=sensor_dir.value()

    print("s1",s1)
    print("s2",s2)

    x=x+1

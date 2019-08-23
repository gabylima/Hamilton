#!/usr/bin/env python3

from ev3dev.ev3 import *
from time import sleep
from os import system
import time
from datetime import datetime, timedelta
from sys import stdout
import paho.mqtt.client as mqtt

# Motores
motor_dir = LargeMotor('outD')
motor_esq = LargeMotor('outB')

garra2 = MediumMotor('outA')

ultra1 = UltrasonicSensor('in3')

ultra2= UltrasonicSensor('in4')

#sensores
sensor_esq = ColorSensor("in1")
sensor_dir = ColorSensor("in2")

sensor_esq.mode = 'COL-REFLECT'
sensor_dir.mode = 'COL-REFLECT'

cont = -1
contador_lado = 0
contLateral = 0
TP = 900
TPDES=300
carga = " "
TPSUB=900
TP_procurar = 400

def Mprocurar():
    stop()
    subirr()
    while True:
        valor_frente = ultra1.value()
        motor_dir.run_forever(speed_sp =TP)
        motor_esq.run_forever(speed_sp=TP)
        CriarCronometro("0:00:02")
        if(carga[1]>= 18 and carga[1] <= 30):
            re()
            re()
            vertiD()
            if(valor_frente >)

def subirr():
    garra2.run_timed(time_sp=1500, speed_sp=TPSUB)
    sleep(0.9)

def mprocurar():
    client = mqtt.Client()
    Conectar(client)
    sleep(0.5)
    subirr()
    try:
        while True:
            valor = ultra1.value()
            print(valor)
            if(valor<=60):
                re()
                re()
                vertiE()
                sleep(0.5)
                stop()
                if(carga[1]>=18 ):
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
                    vertiD()
                    sleep(0.5)
                    if(valor <= 80 and carga[1] <= 20):
                        stop()
                        re()
                        giroE()
                        sleep(1.5)
                        stop()
                    elif(valor <= 80 and carga[1]>=40):
                        stop()
                        re()
                        giroE()
                        sleep(1.5)
                        stop()

            motor_dir.run_forever(speed_sp=TP_procurar)
            motor_esq.run_forever(speed_sp=TP_procurar)
    except KeyboardInterrupt:
            stop()

def vertiE():
    motor_esq.run_to_rel_pos(position_sp=-460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

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
    client.disconnect()

def descerr():

    garra2.run_timed(time_sp=1500, speed_sp=-TPDES)
    sleep(0.9)

def vertiD():
    motor_esq.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-460, speed_sp=400, stop_action="hold")
    sleep(0.5)

def mre():
    motor_esq.run_to_rel_pos(position_sp=-700, speed_sp=1000, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-700, speed_sp=1000, stop_action="hold")
    sleep(0.5)

def stop():
    # O robô para de se mover com os dois motores ao mesmo tempo

    motor_esq.stop()
    motor_dir.stop()
    sleep(0.5)

def frente():

    motor_esq.run_to_rel_pos(position_sp=160, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=160, speed_sp=400, stop_action="hold")
    sleep(0.3)

def re():

    motor_esq.run_to_rel_pos(position_sp=-60, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-60, speed_sp=400, stop_action="hold")
    sleep(0.3)

def CriarCronometro(hora):
    tempo = timedelta(seconds=0)
    while True:
        if str(tempo) ==hora:
            Sound.beep()
            global cont
            cont = cont +1
            break
        tempo = tempo + timedelta(seconds=1)
        sleep(1)

def Principal():
    global contador_lado
    while True:

        motor_dir.run_forever(speed_sp=TP)
        motor_esq.run_forever(speed_sp=TP)

        if cont == -1:
            CriarCronometro('0:00:02')
        else:
            CriarCronometro('0:00:04')

        if cont % 2 == 0:
            TurnoE()
            contador_lado = contador_lado + 1
        else:
            TurnoD()
            contador_lado = contador_lado + 1
        if contador_lado == 3:
            stop()
            Sound.beep()
            break

def giroE():

    motor_esq.run_to_rel_pos(position_sp=-500, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=500, speed_sp=400, stop_action="hold")
    sleep(0.5)

def giroD():
    motor_esq.run_to_rel_pos(position_sp=500, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-500, speed_sp=400, stop_action="hold")
    sleep(0.5)

def TurnoE():
    stop()
    re()
    sleep(1.4)
    giroE()
    sleep(1.4)
    frente()
    frente()
    frente()
    sleep(1.4)
    giroE()
    sleep(1.4)

def TurnoD():
    stop()
    re()
    sleep(1.4)
    giroD()
    sleep(1.4)
    frente()
    frente()
    frente()
    sleep(1.4)
    giroD()
    sleep(1.4)

def CronometroLateral2(hora):
    tempo = timedelta(seconds=0)

    global contLateral

    while True:
        if str(tempo) == hora:
            re()
            re()
            sleep(1.0)
            giroE()
            sleep(0.5)
            contLateral=contLateral+1
            print(contLateral)
            break

        tempo = tempo + timedelta(seconds=1)
        sleep(1)

def Ajuste():

    giroD()
    sleep(1.0)
    frente()

def sala3():
    descerr()

    Principal() #varre no meio

    VarreduraLateral() #varre nas bordas e procura o triângulo

    Ajuste()

    mprocurar()

    #depois varrer mais uma vez na lateral e depositar

def VarreduraLateral():
    try:
        while contLateral<= 5:
            motor_esq.run_forever(speed_sp=TP)
            motor_dir.run_forever(speed_sp=TP)
            CronometroLateral2('0:00:07')

    except KeyboardInterrupt:
        stop()

#VarreduraLateral()

sala3()
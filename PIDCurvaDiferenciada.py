#!/usr/bin/env python3

from ev3dev.ev3 import *
from time import sleep
from os import system
from simple_pid import PID
from csv import*

#CodigoRodando, fazendo tudo exceto curva que tem um gap após ela.




KP = 12
KI = 0
KD = 0
TP = 250.0

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



def calibragem(button2):
    print('posicione os sensores de cor na superficie branca e aperte um botao')
    try:
        while True:
            if button2.enter:
                   print('RODRIGO')
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
    lista = []
    try:
        while True:

            dif = sensor_esq.value() - sensor_dir.value()
            control = pid(dif)



            # condições para evitar ultrapassar o valor máximo do motor

            if (control > 700):
                control = 700
            elif (control < -700) :
                control= -700

            print('dif:' + str(dif))
            print(control)

            motor_esq.run_forever(speed_sp=TP + control)
            motor_dir.run_forever(speed_sp=TP - control)

            lista.append(round(control))

            # offset = 5  # margem de erro para que ele fique reto na linha
            # erro = ( + offset)  # Calcula o erro para que ele sempre siga a linha preta
            # p = kp * erro  # constante proporcionalss
            # # anda de acordo com o erro calculado





    except KeyboardInterrupt:

        motor_dir.stop()
        motor_esq.stop()

        print('aqui {}'.format(lista[0]))
        arq = open('caliii.csv', 'w')
        for i in lista:
            arq.write(str(i)+',\n')
        arq.close()


def menu():
    # Faz a conexão entre o usuario e robô onde ele pode escolher entre
    # calibrar(botão Direito) os valores da pista e rodar(botão Esquerdo) o programa

    button2 = Button()
    print("<< Calibrar |Iniciar >>")
    SP =0;
    while True:
        if button2.left:
            system("clear")
            SP = calibragem(button2)
            print('SP atualizado')
            sleep(0.01)
            print("<< Calibrar |Iniciar >>")

        elif button2.right:
            system("clear")
            executar(TP, SP)
            break
c
menu()



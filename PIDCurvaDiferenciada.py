#!/usr/bin/env python3

from ev3dev.ev3 import *
from time import sleep
from os import system
from simple_pid import PID
from csv import*

#CodigoRodando, fazendo tudo exceto curva que tem um gap após ela.




KP = 11
KI = 0
KD = 0
TP = 280.0
VALOR_MAX_CONTROL = 1000 - TP

# kp=6 ki=0.5 kd=10 tp=250.0

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

sen_esq= ColorSensor("in3")
sen_dir = ColorSensor("in4")

# Put the color sensor into RGB mode. * 0=unknown, 1=black, 2=blue, 3=green, 4=yellow, 5=red, 6=white, 7=brown
sensor_esq.mode = 'COL-REFLECT'
sensor_dir.mode = 'COL-REFLECT'

sen_esq.mode= 'COL-COLOR'
sen_dir.mode = 'COL-COLOR'


def mfrente():
    motor_dir.run_to_rel_pos(position_sp=250, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=250, speed_sp=400, stop_action="hold")
    sleep(0.9)

def mgiroesq():
    motor_esq.run_to_rel_pos(position_sp=-450, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=450, speed_sp=400, stop_action="hold")
    sleep(0.9)

def mgirodi():
    motor_esq.run_to_rel_pos(position_sp=540, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-540, speed_sp=400, stop_action="hold")
    sleep(0.9)

def tras():
    # Função para que o robô ande para a frente por ângulo

    motor_dir.run_to_rel_pos(position_sp=-240, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-240, speed_sp=400, stop_action="hold")
    sleep(0.5)


def frenteMenor():
    # Função que será utilizada para ajudar no auxilio da finalização dos obstáculo.

    motor_dir.run_to_rel_pos(position_sp=-80, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-80, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_dir.run_to_rel_pos(position_sp=-80, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-80, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_dir.run_to_rel_pos(position_sp=-70, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-70, speed_sp=400, stop_action="hold")
    sleep(0.5)


def giroDir():
    # Faz com que o robô ande e se ajuste na linha para a direita

    motor_esq.run_to_rel_pos(position_sp=-360, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=360, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_dir.run_to_rel_pos(position_sp=180, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-180, speed_sp=400, stop_action="hold")
    sleep(0.5)


def giroEsq():
    # Faz com quê o robo gire para á esquerda

    motor_esq.run_to_rel_pos(position_sp=360, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-360, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=180, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-180, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=360, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-360, speed_sp=400, stop_action="hold")
    sleep(0.5)


def frente():
    # Faz com que o robô ande pare trás (positivo)

    motor_dir.run_to_rel_pos(position_sp=40, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=40, speed_sp=400, stop_action="hold")
    sleep(0.5)


def stop():
    # O robô para de se mover com os dois motores ao mesmo tempo

    motor_esq.stop()
    motor_dir.stop()
    sleep(0.5)

def twoverde():
    # Função para quando o robô vê dois verdes ao mesmo tempo com os dois sensores de cor

    stop()
    giroEsq()

    motor_esq.run_to_rel_pos(position_sp=120, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-120, speed_sp=400, stop_action="hold")
    sleep(0.5)

def verde():

        #Função para executar 90º de acordo com qual sensor ele vê o verde

        sensor_esq.mode = 'COL-COLOR'
        sensor_dir.mode = 'COL-COLOR'

        verdeE = sensor_esq.value()
        verdeD = sensor_dir.value()


        if((verdeE==3) and (verdeD==3)):
        #condição para verificar se há dois verdes

                twoverde()

        elif(verdeE==3): #condição caso ele veja verde apenas com o sensor esquerdo

                #Primeiro anda um pouco para trás para verifica se a linha de tras é branca ou preta

                """
                        TAVA  70 , COLOCOU 80 e melhorou, agora retornou para 70  !
                """
                motor_dir.run_to_rel_pos(position_sp=70, speed_sp=400, stop_action="hold")
                motor_esq.run_to_rel_pos(position_sp=70, speed_sp=400, stop_action="hold")
                sleep(0.5)

                stop()

                if ((sensor_esq.value() == 1 ) and (sensor_dir.value() == 6)):

                        motor_esq.run_to_rel_pos(position_sp=-120, speed_sp=400, stop_action="hold")
                        motor_dir.run_to_rel_pos(position_sp=120, speed_sp=400, stop_action="hold")
                        sleep(0.5)

                        motor_dir.run_to_rel_pos(position_sp=-150, speed_sp=400, stop_action="hold")
                        motor_esq.run_to_rel_pos(position_sp=-150, speed_sp=400, stop_action="hold")



                elif (sensor_esq.value() == 1):
                #caso a linha atras seja preta ,ele compensa o que andou para tras para ignorar esse verde ( verde pós preto )

                        motor_dir.run_to_rel_pos(position_sp=-150, speed_sp=400, stop_action="hold")
                        motor_esq.run_to_rel_pos(position_sp=-150, speed_sp=400, stop_action="hold")

                        sleep(0.5)

                elif(sensor_esq.value() == 6):
                #caso  a linha atras seja branca , quer dizer que é um verde normal , ou seja ele executa a funça normalmente

                        Sound.beep()

                        motor_esq.run_to_rel_pos(position_sp=-50, speed_sp=400, stop_action="hold")
                        motor_dir.run_to_rel_pos(position_sp=-50, speed_sp=400, stop_action="hold")
                        sleep(0.5)

                        motor_dir.run_to_rel_pos(position_sp=-120, speed_sp=400, stop_action="hold")
                        motor_esq.run_to_rel_pos(position_sp=-120, speed_sp=400, stop_action="hold")
                        sleep(0.5)

                        motor_esq.run_to_rel_pos(position_sp=360, speed_sp=400, stop_action="hold")
                        motor_dir.run_to_rel_pos(position_sp=-360, speed_sp=400, stop_action="hold")
                        sleep(0.5)


        elif(verdeD==3):

                motor_dir.run_to_rel_pos(position_sp=80, speed_sp=200, stop_action="hold")
                motor_esq.run_to_rel_pos(position_sp=80, speed_sp=200, stop_action="hold")
                sleep(0.5)

                stop()

                if ((sensor_dir.value() == 1 ) and (sensor_esq.value() == 6)):

                        motor_esq.run_to_rel_pos(position_sp=120, speed_sp=400, stop_action="hold")
                        motor_dir.run_to_rel_pos(position_sp=-120, speed_sp=400, stop_action="hold")
                        sleep(0.5)

                        motor_dir.run_to_rel_pos(position_sp=-150, speed_sp=400, stop_action="hold")
                        motor_esq.run_to_rel_pos(position_sp=-150, speed_sp=400, stop_action="hold")


                elif (sensor_dir.value() == 1):
                #caso a linha atras seja preta ,ele compensa o que andou para tras para ignorar esse verde ( verde pós preto )

                        motor_dir.run_to_rel_pos(position_sp=-150, speed_sp=400, stop_action="hold")
                        motor_esq.run_to_rel_pos(position_sp=-150, speed_sp=400, stop_action="hold")

                        sleep(0.5)

                elif(sensor_dir.value() == 6) :
                #caso  a linha atras seja branca , quer dizer que é um verde normal , ou seja ele executa a funça normalmente

                        Sound.beep()

                        motor_esq.run_to_rel_pos(position_sp=-50, speed_sp=400, stop_action="hold")
                        motor_dir.run_to_rel_pos(position_sp=-50, speed_sp=400, stop_action="hold")
                        sleep(0.5)

                        motor_esq.run_to_rel_pos(position_sp=-120, speed_sp=400, stop_action="hold")
                        motor_dir.run_to_rel_pos(position_sp=-120, speed_sp=400, stop_action="hold")
                        sleep(0.5)

                        motor_dir.run_to_rel_pos(position_sp=360, speed_sp=400, stop_action="hold")
                        motor_esq.run_to_rel_pos(position_sp=-360, speed_sp=400, stop_action="hold")
                        sleep(0.5)



        sensor_esq.mode = 'COL-REFLECT'
        sensor_dir.mode = 'COL-REFLECT'

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

def executar(TP, SP):
    pid = PID(KP, KI, KD, setpoint=SP)
    lista = []

    try:
        while True:

            dif = sensor_esq.value() - sensor_dir.value()
            control = pid(dif)

            # condições para evitar ultrapassar o valor máximo do motor

            if (control > VALOR_MAX_CONTROL):
                control = VALOR_MAX_CONTROL
            elif (control < -VALOR_MAX_CONTROL) :
                control= -VALOR_MAX_CONTROL

            #print('dif:' + str(dif))
            #print(control)

            motor_esq.run_forever(speed_sp=TP + control)
            motor_dir.run_forever(speed_sp=TP - control)

            lista.append(round(control))
            #if sen_dir.value() == 3 and sen_esq.value() == 6:
                #stop()
                #frente()
                #mfrente()
                #mgiroesq()

                # O que é isso???
                #if (sen_esq.value() == 1 and sen_dir.value() == 1) or (sen_esq.value() == 1 and sen_dir.value() == 6):
                    #motor_esq.run_to_rel_pos(position_sp=-450, speed_sp=400, stop_action="hold")
                    #motor_dir.run_to_rel_pos(position_sp=450, speed_sp=400, stop_action="hold")
                    #sleep(0.5)


                    #motor_esq.run_to_rel_pos(position_sp=-450, speed_sp=400, stop_action="hold")
                    #motor_dir.run_to_rel_pos(position_sp=450, speed_sp=400, stop_action="hold")
                    #sleep(0.5)
            #elif sen_dir.value() == 6 and sen_esq.value() == 3:
                #stop()
                #mfrente()
                #mgirodi()
                # frente()

            #elif dif > 0:
                #print('verde para direita')

            # offset = 5  # margem de erro para que ele fique reto na linha
            # erro = ( + offset)  # Calcula o erro para que ele sempre siga a linha preta
            # p = kp * erro  # constante proporcionalss
            # # anda de acordo com o erro calculado





    except KeyboardInterrupt:

        motor_dir.stop()
        motor_esq.stop()

        print('aqui {}'.format(lista[0]))
        arq = open('rocha.csv', 'w')
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
            print("<< Calibrar | Iniciar >>")

        elif button2.right:
            system("clear")
            executar(TP, SP)
            break

menu()
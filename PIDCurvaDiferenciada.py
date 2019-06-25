#!/usr/bin/env python3

from ev3dev.ev3 import *
from time import sleep
from os import system
from simple_pid import PID
from csv import*

KP = 11 #modifiquei o kp: tava 12
KI = 0
KD = 0
TP = 280.0 #modifiquei tp: tava 280
VALOR_MAX_CONTROL = 1000 - TP  # antes tava 1000
CORRECAO_MOTOR = 10
system('setfont Lat15-TerminusBold14')

# Motores
motor_dir = LargeMotor('outB')
motor_esq = LargeMotor('outD')
sensor_esq = ColorSensor("in1")
sensor_dir = ColorSensor("in2")


# Sensores
# Modo RGB. * 0=desconhecido, 1=preto, 2=azul, 3=verde, 4=amarelo, 5=vermelho, 6=branco, 7=marrom
sensor_esq.mode = 'COL-REFLECT'  #
sensor_dir.mode = 'COL-REFLECT'  #


# Sensor ultrassonico
us = UltrasonicSensor()
ultra1 = UltrasonicSensor('in3')
ultra2= UltrasonicSensor('in4')



#s3 =ColorSensor("in3")
#s4 =ColorSensor("in4")

#s3.mode = 'COL-REFLECT'
#s4.mode = 'COL-REFLECT'

def obstaculo():
    TPO = 250
    KP = 1
    KI = 0
    KD = 0
    SPO = 25
    R=440
    VALOR_MAX_CONTROL = R - TPO  # é o que define sua potencia para girar, se aumentar o TP terá que aumentar a constante que
    # subtrai o TP

    pid = PID(KP, KI, KD, setpoint=SPO)
    lista1 = []
    try:
        while True:
            valor = ultra2.value()
            control = pid(valor)
            lista1.append(round(control))

            if (control > VALOR_MAX_CONTROL):
                control = VALOR_MAX_CONTROL
            elif (control < -VALOR_MAX_CONTROL):
                control = -VALOR_MAX_CONTROL

            motor_esq.run_forever(speed_sp=TPO - control)
            motor_dir.run_forever(speed_sp=TPO + control)
            e = motor_esq.count_per_rot
            d = motor_dir.count_per_rot

    except KeyboardInterrupt:

        motor_dir.stop()
        motor_esq.stop()

        arq = open('obstaculo.csv', 'w')

        for i in lista1:
            arq.write(str(i) + ',\n')
        arq.close()

def mfrente():
    motor_dir.run_to_rel_pos(position_sp=90, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=90, speed_sp=400, stop_action="hold")
    sleep(0.9)

def mgirodi():
    motor_esq.run_to_rel_pos(position_sp=-450, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=450, speed_sp=400, stop_action="hold")
    sleep(0.9)

def mgiroesq():
    motor_esq.run_to_rel_pos(position_sp=540, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-540, speed_sp=400, stop_action="hold")
    sleep(0.9)

def GirarAteVerObstaculo():
    while (ultra2.value() >328):
        giroDir()

def GirarAteNVerObstaculo():

    while (ultra2.value() < 1000 ):
        giroDir()

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


    motor_dir.run_to_rel_pos(position_sp=180, speed_sp=200, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-180, speed_sp=200, stop_action="hold")

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

def verde2():

    sensor_esq.mode = 'COL-COLOR'
    sensor_dir.mode = 'COL-COLOR'

    verdeE = sensor_esq.value()
    verdeD = sensor_dir.value()

    if (verdeE == 3 and verdeD==6):
        mfrente()                      #modifiquei adcionando o mfrente
        mgiroesq()
        mfrente()
        Sound.beep()

    elif(verdeD== 3 and verdeE==6):
        mfrente()                   #modifiquei adcionando o mfrente
        mgirodi()
        mfrente()
        Sound.beep()

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
            if ultra1.value() <= 100:

                stop()
                Sound.beep()
                #GirarAteVerObstaculo() #tirei temporariamente
                #obstaculo() #tirei temporariamente

            dif = sensor_esq.value() - sensor_dir.value()
            control = pid(dif)
            #print("control {}".format(control))


            # codigo para identificar verde

            #direito no branco = 86 ou 87

            # direito no preto = 7

            # direito no verde = 12

            #branco para verde = 62

            # esquerdo no branco = 79

            # esquerdo no preto = 7

            # esquerdo no verde = 9

            # branco para verde = 67

            if (sensor_dir.value() == 10 or sensor_esq.value() == 9):
                verde2()


            # condições para evitar ultrapassar o valor máximo do motor
            if (control > VALOR_MAX_CONTROL):
                control = VALOR_MAX_CONTROL
            elif (control < -VALOR_MAX_CONTROL) :
                control= -VALOR_MAX_CONTROL









            #condição para não sair da linha, usando o sensor do meio

            motor_esq.run_forever(speed_sp=TP - control)
            motor_dir.run_forever(speed_sp=TP + control + CORRECAO_MOTOR)


        lista.append(round(control))


    except KeyboardInterrupt:

        motor_dir.stop()
        motor_esq.stop()

        arq = open('rocha.csv', 'w')
        for i in lista:
            arq.write(str(i)+',\n')
        arq.close()

def menu():
    # Faz a conexão entre o usuario e robô onde ele pode escolher entre
    # calibrar(botão Direito) os valores da pista e rodar(botão Esquerdo) o programa

    button2 = Button()
    print("<< Calibrar | Iniciar >>")

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
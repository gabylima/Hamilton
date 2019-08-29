#!/usr/bin/env python3

from ev3dev.ev3 import *
from time import sleep
from os import system
from simple_pid import PID
import paho.mqtt.client as mqtt
from datetime import datetime, timedelta
import  struct
from csv import *

sair = False
system('setfont Lat15-TerminusBold14')# estilização

KP = 12 #modifiquei o kp: tava 12
KI = 0
KD = 0
TP = 210.0 #modifiquei tp: tava 280
VALOR_MAX_CONTROL = 1000 - TP
CORRECAO_MOTOR = 0
dicionario = ''
# Motores
motor_dir = LargeMotor('outD')
motor_esq = LargeMotor('outB')
#garra =MediumMotor('outA')
garra2 = MediumMotor('outA')


#sensores
sensor_esq = ColorSensor("in1")
sensor_dir = ColorSensor("in2")

sensor_esq.mode = 'COL-REFLECT'
sensor_dir.mode = 'COL-REFLECT'

# Sensor ultrassonico
ultra1 = UltrasonicSensor('in3')
ultra2= UltrasonicSensor('in4')

# Modo Color. * 0=desconhecido, 1=preto, 2=azul, 3=verde, 4=amarelo, 5=vermelho, 6=branco, 7=marrom

def calibragem(button2):
    esquerdo = 0

    direito = 0

    print("-----------------")
    print("---<<BRANCOS>>---")
    print("-----------------")
    print("")

    print("Pressione o botao do meio")

    try:
        while True:

            #Calibra primeiro o SP
            if button2.enter:
                print('ola')
                di = sensor_dir.value()
                esq = sensor_esq.value()

                sp = esq - di

                sleep(2)
                break
        system("clear")
        sleep(1)

        print("----------------")
        print("---<<VERDE>>---")
        print("----ESQUERDO----")
        print("----------------")
        print("")

        print("Pressione o botao! ")
        print("")

        while True:

            if button2.enter:
                esquerdo = sensor_esq.value()
                break

        sleep(2)
        print("---------------")
        print("---<<VERDE>>---")
        print("----DIREITO----")
        print("---------------")
        print("")

        print("Pressione o botao do meio ")
        print("")
        while True:
            if button2.enter:
                direito = sensor_dir.value()
                break

        global dicionario
        dicionario = {'esquerdo': esquerdo, 'direito':direito}
        print("LET'S GO!")
        system("clear")
        sleep(2)

        return sp

    except KeyboardInterrupt:
        motor_dir.stop()
        motor_esq.stop()
def stop():
    # O robô para de se mover com os dois motores ao mesmo tempo

    motor_esq.stop()
    motor_dir.stop()
    sleep(0.5)
def twoverde():
    # Função para quando o robô vê dois verdes ao mesmo tempo com os dois sensores de cor

    stop()

    motor_dir.run_to_rel_pos(position_sp=-450, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=450, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_dir.run_to_rel_pos(position_sp=-450, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=450, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_dir.run_to_rel_pos(position_sp=-450, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=450, speed_sp=400, stop_action="hold")
    sleep(1.0)

    motor_dir.run_to_rel_pos(position_sp=-450, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=450, speed_sp=400, stop_action="hold")
    sleep(0.5)



def twoverde2():
    stop()

    motor_esq.run_to_rel_pos(position_sp=-360, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=360, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=-180, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=180, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=-360, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=360, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=-120, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=120, speed_sp=400, stop_action="hold")
    sleep(0.5)
def verde():
    sensor_esq.mode = 'COL-COLOR'
    sensor_dir.mode = 'COL-COLOR'

    verdeE = sensor_esq.value()
    verdeD = sensor_dir.value()

    if ((verdeE == 3) and (verdeD == 3)):
        # condição para verificar se há dois verdes

        twoverde()
    elif (verdeE == 3):  # condição caso ele veja verde apenas com o sensor esquerdo

        # Primeiro anda um pouco para trás para verifica se a linha de tras é branca ou preta

        """
            TAVA  70 , COLOCOU 80 e melhorou, agora retornou para 70  !
        """
        motor_dir.run_to_rel_pos(position_sp=-70, speed_sp=400, stop_action="hold")
        motor_esq.run_to_rel_pos(position_sp=-70, speed_sp=400, stop_action="hold")
        sleep(0.5)

        stop()

        if ((sensor_esq.value() == 1) and (sensor_dir.value() == 6)):
            motor_esq.run_to_rel_pos(position_sp=120, speed_sp=400, stop_action="hold")
            motor_dir.run_to_rel_pos(position_sp=-120, speed_sp=400, stop_action="hold")
            sleep(0.5)

            motor_dir.run_to_rel_pos(position_sp=150, speed_sp=400, stop_action="hold")
            motor_esq.run_to_rel_pos(position_sp=150, speed_sp=400, stop_action="hold")

        elif (sensor_esq.value() == 1):
            # caso a linha atras seja preta ,ele compensa o que andou para tras para ignorar esse verde ( verde pós preto )

            motor_dir.run_to_rel_pos(position_sp=150, speed_sp=400, stop_action="hold")
            motor_esq.run_to_rel_pos(position_sp=150, speed_sp=400, stop_action="hold")

            sleep(0.5)

        elif (sensor_esq.value() == 6):
            # caso  a linha atras seja branca , quer dizer que é um verde normal , ou seja ele executa a funça normalmente

            Sound.beep()

            motor_esq.run_to_rel_pos(position_sp=50, speed_sp=400, stop_action="hold")
            motor_dir.run_to_rel_pos(position_sp=50, speed_sp=400, stop_action="hold")
            sleep(0.5)

            motor_dir.run_to_rel_pos(position_sp=120, speed_sp=400, stop_action="hold")
            motor_esq.run_to_rel_pos(position_sp=120, speed_sp=400, stop_action="hold")
            sleep(0.5)

            motor_esq.run_to_rel_pos(position_sp=-360, speed_sp=400, stop_action="hold")
            motor_dir.run_to_rel_pos(position_sp=360, speed_sp=400, stop_action="hold")
            sleep(0.5)

    elif (verdeD == 3):

        motor_dir.run_to_rel_pos(position_sp=-80, speed_sp=200, stop_action="hold")
        motor_esq.run_to_rel_pos(position_sp=-80, speed_sp=200, stop_action="hold")
        sleep(0.5)

        stop()

        if ((sensor_dir.value() == 1) and (sensor_esq.value() == 6)):

            motor_esq.run_to_rel_pos(position_sp=-120, speed_sp=400, stop_action="hold")
            motor_dir.run_to_rel_pos(position_sp=120, speed_sp=400, stop_action="hold")
            sleep(0.5)

            motor_dir.run_to_rel_pos(position_sp=150, speed_sp=400, stop_action="hold")
            motor_esq.run_to_rel_pos(position_sp=150, speed_sp=400, stop_action="hold")


        elif (sensor_dir.value() == 1):
                # caso a linha atras seja preta ,ele compensa o que andou para tras para ignorar esse verde ( verde pós preto )

            motor_dir.run_to_rel_pos(position_sp=150, speed_sp=400, stop_action="hold")
            motor_esq.run_to_rel_pos(position_sp=150, speed_sp=400, stop_action="hold")

            sleep(0.5)

        elif (sensor_dir.value() == 6):
                # caso  a linha atras seja branca , quer dizer que é um verde normal , ou seja ele executa a funça normalmente

            Sound.beep()

            motor_esq.run_to_rel_pos(position_sp=50, speed_sp=400, stop_action="hold")
            motor_dir.run_to_rel_pos(position_sp=50, speed_sp=400, stop_action="hold")
            sleep(0.5)

            motor_esq.run_to_rel_pos(position_sp=120, speed_sp=400, stop_action="hold")
            motor_dir.run_to_rel_pos(position_sp=120, speed_sp=400, stop_action="hold")
            sleep(0.5)

            motor_dir.run_to_rel_pos(position_sp=-360, speed_sp=400, stop_action="hold")
            motor_esq.run_to_rel_pos(position_sp=360, speed_sp=400, stop_action="hold")
            sleep(0.5)
    sensor_esq.mode = 'COL-REFLECT'
    sensor_dir.mode = 'COL-REFLECT'

def executar(sp):
    pid = PID(KP, KI, KD, setpoint=sp)

    try:

        while True:
            dif = sensor_esq.value() - sensor_dir.value()
            control = pid(dif)

            if (dicionario["esquerdo"]- 2 < sensor_dir.value() < dicionario["esquerdo"]+ 2):
                verde()

            elif (dicionario["direito"]- 2 < sensor_esq.value() < dicionario["direito"] + 2):
                verde()

            if (control > VALOR_MAX_CONTROL):
                control = VALOR_MAX_CONTROL
            elif (control < -VALOR_MAX_CONTROL):
                control = -VALOR_MAX_CONTROL

            # motores com o PID
            motor_esq.run_forever(speed_sp=TP - control + CORRECAO_MOTOR)
            motor_dir.run_forever(speed_sp=TP + control)


    except KeyboardInterrupt:
        motor_esq.stop()
        motor_dir.stop()

def rampa(setpoint):
    KPR = 10  # modifiquei o kp: tava 12
    KIR = 0
    KDR = 0
    TPR = 400.0  # modifiquei tp: tava 280

    V_MAX_MOTOR = 1000
    VALOR_MAX_CONTROL = V_MAX_MOTOR - TPR

    pid = PID(KPR, KIR, KDR, setpoint=setpoint)
    # print(TPR, KPR)

    try:

        while True:

            valor = sensor_esq.value() - sensor_dir.value()
            control = pid(valor)

            if (control > VALOR_MAX_CONTROL):
                control = VALOR_MAX_CONTROL
            elif (control < -VALOR_MAX_CONTROL):
                control = -VALOR_MAX_CONTROL

            motor_esq.run_forever(speed_sp=TPR - control)
            motor_dir.run_forever(speed_sp=TPR + control)

    except KeyboardInterrupt:

        motor_dir.stop()
        motor_esq.stop()

botao = Button()
setpoint = calibragem(botao)
executar(setpoint)
#rampa(setpoint)

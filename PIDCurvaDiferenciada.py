#!/usr/bin/env python3

from ev3dev.ev3 import *
from time import sleep
from os import system
from simple_pid import PID
import paho.mqtt.client as mqtt
import  struct
from csv import *

#system('setfont Lat15-TerminusBold14') # estilização
KP = 18#modifiquei o kp: tava 12
KI = 0
KD = 0
TP = 210.0 #modifiquei tp: tava 280
VALOR_MAX_CONTROL = 1000 - TP
CORRECAO_MOTOR = 0
carga =" "
TPDES=900
TPDES1=100
cont = 0
sensor_cor=0

# Motores
motor_dir = LargeMotor('outD')
motor_esq = LargeMotor('outB')
#garra =MediumMotor('outA')
garra = LargeMotor('outA')
garra2 = MediumMotor('outC')


#sensores
sensor_esq = ColorSensor("in1")
sensor_dir = ColorSensor("in2")

sensor_esq.mode = 'COL-REFLECT'
sensor_dir.mode = 'COL-REFLECT'

# Sensor ultrassonico
ultra1 = UltrasonicSensor('in3')
ultra2= UltrasonicSensor('in4')

# Modo Color. * 0=desconhecido, 1=preto, 2=azul, 3=verde, 4=amarelo, 5=vermelho, 6=branco, 7=marrom

def reto():
    motor_esq.run_to_rel_pos(position_sp=540, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=540, speed_sp=400, stop_action="hold")
    sleep(0.5)

def vira():
    motor_esq.run_to_rel_pos(position_sp=-460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=-460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

def Restart():
    button = Button()
    if button.left:
        while True:
            stop()
            if button.right:
                break

def sala3():
    descer()
    try:
        while True:
            ul = ultra1.value()
            if (ul > 30) :
                #motor_esq.run_forever(speed_sp=TP)
                #motor_dir.run_forever(speed_sp=TP)
                reto()
                sleep(0.9)
                subir()
                sleep(0.9)
                descer()

            elif(ul<=30):
                stop()

    except KeyboardInterrupt:

        motor_dir.stop()
        motor_esq.stop()

def VerificaSala3(c):
    if (ultra2.value() >= 300 and carga[1] <= 4 and c >= 9):
        return 1
    elif ((ultra2.value() >= 30 and ultra2.value() <= 78) and carga[1] >= 70 and c >= 9):
        return 1
    elif (ultra2.value() >= 300 and carga[1] >= 70):
        global cont
        cont = 0

def on_connect(client, userdata, flags,message):
    client.subscribe("topic/teste")

def on_message(client, userdata, message):
    global carga
    carga = unpack('iiii', message.payload)

def EgiroObs():
    motor_esq.run_to_rel_pos(position_sp=-120, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=120, speed_sp=400, stop_action="hold")
    sleep(0.5)

def DgiroObs():
    motor_esq.run_to_rel_pos(position_sp=540, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-540, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=180, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-180, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=90, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-90, speed_sp=400, stop_action="hold")
    sleep(0.5)

def PosObstaculo():
    stop()
    mfrente()
    mfrente()
    DgiroObs()
    mtras()
    mtras()
    stop()

def Conectar(client):
    client.connect("10.42.0.183", 1883, 60)
    client.on_connect = on_connect
    client.on_message = on_message
    client.loop_start()

def Desconectar(client):
    client.loop_stop()
    client.disconnect()

def mfrentemenor():
    #volta so um pouco para tras
    motor_dir.run_to_rel_pos(position_sp=75, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=75, speed_sp=400 + CORRECAO_MOTOR, stop_action="hold")
    sleep(0.5)

def EgiroVerde():
    #gira para a esquerda
    motor_esq.run_to_rel_pos(position_sp=-540, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=540, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=-180, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=180, speed_sp=400, stop_action="hold")
    sleep(0.5)

def DgiroVerde():

    #gira para a direita
    motor_esq.run_to_rel_pos(position_sp=540, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-540, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=180, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-180, speed_sp=400, stop_action="hold")
    sleep(0.5)

def mtras():
    #volta um pouco para tras

    motor_dir.run_to_rel_pos(position_sp=-100, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-100, speed_sp=400 + CORRECAO_MOTOR, stop_action="hold")
    sleep(0.5)

def obstaculo():
    global carga
    KPO = 6
    TPO = 100
    KIO = 0
    KDO = 0
    SPO = 15
    V_MAX_MOTOR=1000
    VALOR_MAX_CONTROL = V_MAX_MOTOR - TPO


    pid = PID(KPO, KIO, KDO, setpoint=SPO)
    lista1 = []
    try:

        while (carga[0] != 1):
            valor = ultra2.value()
            control = pid(valor)
            lista1.append(round(control))

            if (control > VALOR_MAX_CONTROL):
                control = VALOR_MAX_CONTROL
            elif (control < -VALOR_MAX_CONTROL):
                control = -VALOR_MAX_CONTROL


            motor_esq.run_forever(speed_sp=TPO)
            motor_dir.run_forever(speed_sp=TPO - control)
        PosObstaculo()


    except KeyboardInterrupt:

        motor_dir.stop()
        motor_esq.stop()

        arq = open('obstaculo.csv', 'w')

        for i in lista1:
            arq.write(str(i) + ',\n')
        arq.close()

def mfrente():
    #ir para frente
    motor_dir.run_to_rel_pos(position_sp=140, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=140, speed_sp=400, stop_action="hold")
    sleep(0.5)

def GirarAteVerObstaculo():

    #vai da um giro de 90

    motor_esq.run_to_rel_pos(position_sp=440, speed_sp=200, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-440, speed_sp=200, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=440, speed_sp=200, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-440, speed_sp=200, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=440, speed_sp=200, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-440, speed_sp=200, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=440, speed_sp=200, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-440, speed_sp=200, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=90, speed_sp=200, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-90, speed_sp=200, stop_action="hold")
    sleep(0.5)

def frenteMenor():
    # Função que será utilizada para ajudar no auxilio da finalização dos obstáculo.

    motor_dir.run_to_rel_pos(position_sp=80, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=80, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_dir.run_to_rel_pos(position_sp=-80, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-80, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_dir.run_to_rel_pos(position_sp=-70, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-70, speed_sp=400, stop_action="hold")
    sleep(0.5)

def stop():
    # O robô para de se mover com os dois motores ao mesmo tempo

    motor_esq.stop()
    motor_dir.stop()
    sleep(0.5)

def twoverde():
    # Função para quando o robô vê dois verdes ao mesmo tempo com os dois sensores de cor

    stop()

    motor_dir.run_to_rel_pos(position_sp=450, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-450, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_dir.run_to_rel_pos(position_sp=450, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-450, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_dir.run_to_rel_pos(position_sp=450, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-450, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_dir.run_to_rel_pos(position_sp=450, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-450, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_dir.run_to_rel_pos(position_sp=450, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-450, speed_sp=400, stop_action="hold")
    sleep(0.5)

def IdentificaVerde():

    sensor_esq.mode = 'COL-COLOR'
    sensor_dir.mode = 'COL-COLOR'

    verdeE = sensor_esq.value()
    verdeD = sensor_dir.value()

    if(verdeE==sensor_cor['esquerdo'] and verdeD==sensor_cor['direito']):
    #se ele ver dois verdes
        twoverde()

    elif (verdeE == sensor_cor['esquerdo'] or verdeD==sensor_cor['direito']):
    #se ele ver algum verde de ambos os lados
        mtras()
        stop()
        if(verdeD== 1 and verdeE ==1):
            #caso ele recue e veja preto nos dois sensores, ignore-os
            mfrente()
        elif (verdeD == 1 and verdeE == 6):
            mfrente()
        elif(verdeD == 6 and verdeE == 1):
            mfrente()
        elif (verdeD== 6 and verdeE ==6):
            #caso ele recue e veja branco nos dois sensores, ele avançará pra frente
            mfrentemenor()
            verdeEs = sensor_esq.value()
            verdeDi = sensor_dir.value()
            stop()
            if (verdeDi == sensor_cor['direito'] and verdeEs == 6):
                # se ele ver verde do lado direito
                mfrente()
                DgiroVerde()
            elif (verdeDi== 6 and verdeEs==sensor_cor['esquerdo']):
                #se ele ver verde do lado esquerdo
                mfrente()
                EgiroVerde()


    sensor_esq.mode = 'COL-REFLECT'
    sensor_dir.mode = 'COL-REFLECT'

def calibragemSP(button2):
    print("<<< BRANCO >>>")
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

def ReObstaculo():
    motor_dir.run_to_rel_pos(position_sp=-40, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-40, speed_sp=400 + CORRECAO_MOTOR, stop_action="hold")
    sleep(0.5)

def descer():
    garra.run_timed(time_sp = 1500, speed_sp=-TPDES1)
    garra2.run_timed(time_sp = 1500, speed_sp=-TPDES)

    #garra.run_to_rel_pos(position_sp=-75, speed_sp=400, stop_action="hold")

def subir():
    garra2.run_timed(time_sp=1500, speed_sp=TPDES)
    garra.run_timed(time_sp=1500, speed_sp=TPDES1)

    #garra.run_to_rel_pos(position_sp=75, speed_sp=400, stop_action="hold")

def DualGreen():
    if(carga[3] == 3 and carga[2]==3):
        twoverde()
        mfrentemenor()
    else:
        mtras()
        stop()
        if(carga[3] == 1 and carga[2] == 1):
            mfrente()
        elif(carga[3] == 1 and carga[2]== 6):
            mfrente()
        elif (carga[3] == 6 and carga[2] == 1):
            mfrente()
        elif(carga[3] == 6 and carga[2] == 6):
            mfrentemenor()
            if(carga[3]==3 and (carga[2]==6 or carga[2]==1)):
                DgiroVerde()
                mfrentemenor()
            elif(carga[2] == 3 and (carga[3]==6 or carga[3]==1)):
                EgiroVerde()
                mfrentemenor()

def executar(TP, SP):
    print(carga)
    pid = PID(KP, KI, KD, setpoint=SP)
    lista = []
    client = mqtt.Client()
    Conectar(client)
    sleep(0.5)
    try:
        while True:
            print('carga:',carga)
            Restart()
            if((ultra2.value()>=36 and ultra2.value()<=80)and(carga[1]<=2)):
                global cont
                cont=cont+1
            s3 = VerificaSala3(cont)
        # parte do obstaculo
            if ultra1.value() <= 50:
                stop()
                Sound.beep()
                ReObstaculo()
                GirarAteVerObstaculo()
                stop()
                client2 = mqtt.Client()
                Conectar(client2)
                obstaculo()
                Desconectar(client2)

        #parte do PID
            dif = sensor_esq.value() - sensor_dir.value()
            control = pid(dif)


        #parte do verde
            #if (carga[3] == 3 or carga[2]== 3):
            #    DualGreen()

        # condições para evitar ultrapassar o valor máximo do motor
            if (control > VALOR_MAX_CONTROL):
                control = VALOR_MAX_CONTROL
            elif (control < -VALOR_MAX_CONTROL) :
                control= -VALOR_MAX_CONTROL

        #motores com o PID
            motor_esq.run_forever(speed_sp=TP - control)
            motor_dir.run_forever(speed_sp=TP + control + CORRECAO_MOTOR)


        #lista.append(round(control))
            if(s3==1):
                break


        sala3()

    except KeyboardInterrupt:

        motor_dir.stop()
        motor_esq.stop()
        print('cont:',cont)
        arq = open('rocha.csv', 'w')
        for i in lista:
            arq.write(str(i)+',\n')
        arq.close()

def CalibrarVerde(button):
    print("<<< VERDE >>>")
    print('aperte o botao esquerdo para calibrar o sensor esquerdo quando estiver no verde')
    print()
    print('aperte o botao direito para calibrar o sensor direito quando estiver no verde')
    print()
    print('aperte o botao do meio pra sair')
    try:
        while True:
            if button.right:
                direito = sensor_dir.value()
                sensor_dir.mode='COL-COLOR'
                cor_direito = sensor_dir.value()
                sensor_dir.mode='COL-REFLECT'

            elif button.left:
                esquerdo = sensor_esq.value()
                sensor_esq.mode='COL-COLOR'
                cor_esquerdo = sensor_esq.value()
                sensor_esq.mode='COL-REFLECT'
            elif button.enter:
                break
            else:
                sleep(0.01)
        global sensor_cor
        dici_total = {'esquerdo':esquerdo,'direito':direito}
        sensor_cor = {'esquerdo':cor_esquerdo,'direito':cor_direito}
        return dici_total


    except KeyboardInterrupt:
        motor_dir.stop()
        motor_esq.stop()

def menu():
    # Faz a conexão entre o usuario e robô onde ele pode escolher entre
    # calibrar(botão Direito) os valores da pista e rodar(botão Esquerdo) o programa

    button2 = Button()
    print("<< Calibrar | Iniciar >>")

    SP = 0;
    verde = 0;
    while True:

        if button2.left:
            system("clear")
            SP = calibragemSP(button2)
            print('SP atualizado:',SP)
            sleep(0.01)
            system("clear")
            #verde = CalibrarVerde(button2)
            #print(verde)
            #print(sensor_cor)
            sleep(0.01)
            print("<< Calibrar | Iniciar >>")


        elif button2.right:
            system("clear")
            executar(TP, SP)
            break

menu()

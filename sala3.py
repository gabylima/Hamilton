#!/usr/bin/env python3

from ev3dev.ev3 import *
from time import sleep
from os import system
from simple_pid import PID
import paho.mqtt.client as mqtt
from csv import*


system('setfont Lat15-TerminusBold14') # estilização
KP = 11 #modifiquei o kp: tava 12
KI = 0
KD = 0
TP = 210.0 #modifiquei tp: tava 280
VALOR_MAX_CONTROL = 1000 - TP
CORRECAO_MOTOR = 10
carga =" "
#TPDES=100
TPDES=300
TPDES1=300

#TPSUB=100
TPSUB=900
TPSUB1=80


# Motores
motor_dir = LargeMotor('outB')
motor_esq = LargeMotor('outD')
#garra = LargeMotor('outA')
#garra2 = MediumMotor('outC')

garra2 = MediumMotor('outA')

#sensores
ultra1 = UltrasonicSensor('in3')
ultra2= UltrasonicSensor('in4')

def subirr():
    garra2.run_timed(time_sp=1500, speed_sp=TPSUB)
    sleep(0.9)

def descer ():
    garra.run_timed(time_sp=1500, speed_sp=TPDES1)
    sleep(0.9)
    garra2.run_timed(time_sp=1500, speed_sp=TPDES)
    sleep(0.9)

def Egiro():
    #gira para a esquerda
    motor_esq.run_to_rel_pos(position_sp=-540, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=540, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=-180, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=180, speed_sp=400, stop_action="hold")
    sleep(0.5)

def Dgiro():

    #gira para a direita
    motor_esq.run_to_rel_pos(position_sp=540, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-540, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=180, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-180, speed_sp=400, stop_action="hold")
    sleep(0.5)

def subir ():
    garra2.run_timed(time_sp=1500, speed_sp=-TPSUB)
    sleep(0.9)
    garra.run_timed(time_sp=1500, speed_sp=-TPSUB1)
    sleep(0.9)
    garra2.run_timed(time_sp=1500, speed_sp=40)

#def frente():
    #motor_dir.run_to_rel_pos(position_sp=140, speed_sp=400, stop_action="hold")
    #motor_esq.run_to_rel_pos(position_sp=140, speed_sp=400, stop_action="hold")
    #sleep(0.5)

#descer()

def movimento():
    motor_esq.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

    motor_esq.run_to_rel_pos(position_sp=300, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=300, speed_sp=400, stop_action="hold")
    sleep(0.5)
'''
descer()
sleep(0.5)
movimento()
sleep(0.5)
subir()

'''
def stop():
    motor_dir.stop()
    motor_esq.stop()

def frente():

    motor_esq.run_to_rel_pos(position_sp=160, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=160, speed_sp=400, stop_action="hold")
    sleep(0.3)

def mdescer():
    garra.run_timed(time_sp=1500, speed_sp=TPDES1)
    sleep(0.9)

def mover():

    try:
        while True:
            valor = ultra1.value()
            if (valor >= 100 and valor <=190):
                stop()
                mdescer()
                sleep(0.5)
                subir()
                sleep(0.5)
                stop()
            if(valor <= 60):
                stop()
                Egiro()

                frente()
                Egiro()
            motor_dir.run_forever(speed_sp=TP)
            motor_esq.run_forever(speed_sp=TP)
    except KeyboardInterrupt:
        motor_esq.stop()
        motor_dir.stop()

def manual():

    try:
        while True:
            valor = ultra1.value()

            frente()

            if (valor >= 100 and valor <= 190 ):
                stop()
                mdescer()
                sleep(0.5)
                subir()
                sleep(0.5)
                stop()

            if (valor <= 60):
                stop()
                Egiro()

                frente()
                Egiro()


    except KeyboardInterrupt:
        motor_esq.stop()
        motor_dir.stop()


#mover()
#manual()

#subirr()
#sleep(0.5)
#subir()

def descerr():

    garra2.run_timed(time_sp=1500, speed_sp=-TPDES)
    sleep(0.9)

def frentee():
    client = mqtt.Client()
    Conectar(client)
    sleep(0.5)

    descerr()

    KP = 12
    TP = 210.0
    KIO = 0
    KD = 0
    SP = 3

    V_MAX_MOTOR = 1000
    VALOR_MAX_CONTROL = V_MAX_MOTOR - TP
    for i in range(0, 30):
        frente()

        pid = PID(KP, KIO, KD, setpoint=SP)

        control = pid(carga[1])

        if (control > VALOR_MAX_CONTROL):
            control = VALOR_MAX_CONTROL
        elif (control < -VALOR_MAX_CONTROL):
            control = -VALOR_MAX_CONTROL

        motor_esq.run_forever(speed_sp=TP - control)
        motor_dir.run_forever(speed_sp=TP + control)

        if (carga[1] >= 18):
            Sound.beep()
            stop()

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







    stop()
    Desconectar(client)

def frenteTri():
    motor_esq.run_to_rel_pos(position_sp=500, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=500, speed_sp=400, stop_action="hold")
    sleep(0.3)

def alinha():
    motor_esq.run_to_rel_pos(position_sp=540, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-540, speed_sp=400, stop_action="hold")
    sleep(0.5)

def gfrente():
    client = mqtt.Client()
    Conectar(client)
    sleep(0.5)
    descerr()
    for i in range(0, 30):
        frente()
        if(carga[1]>=18):
            Sound.beep()
            stop()
            sleep(0.5)
            frente()
            giroD()
            frente()
            giroD()
            frente()
            giroD()
            sleep(0.5)
            break


    stop()
    Desconectar(client)

def re():
    motor_esq.run_to_rel_pos(position_sp=-60, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-60, speed_sp=400, stop_action="hold")
    sleep(0.5)

def reFinal():
    motor_esq.run_to_rel_pos(position_sp=-200, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-200, speed_sp=400, stop_action="hold")
    sleep(0.5)

def giroE():

    motor_esq.run_to_rel_pos(position_sp=500, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-500, speed_sp=400, stop_action="hold")
    sleep(0.5)

def frentinha():
    motor_esq.run_to_rel_pos(position_sp=500, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=500, speed_sp=400, stop_action="hold")
    sleep(0.5)

def giroD():
    motor_esq.run_to_rel_pos(position_sp=-500, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=500, speed_sp=400, stop_action="hold")
    sleep(0.5)

def girop():
    motor_esq.run_to_rel_pos(position_sp=-500, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=500, speed_sp=400, stop_action="hold")
    sleep(0.5)

def on_connect(client, userdata, flags, message):
    client.subscribe("topic/teste")

def on_message(client, userdata, message):
    global carga
    carga = unpack('iii', message.payload)

def Conectar(client):
    client.connect("10.42.0.183", 1883, 60)
    client.on_connect = on_connect
    client.on_message = on_message
    client.loop_start()

def Desconectar(client):
    client.loop_stop()
    client.disconnect()

def procurar():
    client = mqtt.Client()
    Conectar(client)
    sleep(0.5)

    re()
    re()
    giroD()
    sleep(1.5)
    giroD()
    sleep(1.5)



    KP= 40
    TP = 210.0
    KIO= 0
    KD = 0
    SP = 1

    V_MAX_MOTOR = 1000
    VALOR_MAX_CONTROL = V_MAX_MOTOR - TP

    pid = PID(KP, KIO, KD, setpoint=SP)


    try:

        while True:
            valor = carga[2]
            control = pid(valor)



            if (control > VALOR_MAX_CONTROL):
                control = VALOR_MAX_CONTROL
            elif (control < -VALOR_MAX_CONTROL):
                control = -VALOR_MAX_CONTROL

            if (carga[2]==1 and carga[1]>=8):
                Sound.beep()
                stop()
                frente()
                sleep(1.5)
                giroE()
                sleep(1.5)
                descerr()
                sleep(1.5)
                reTri()
                sleep(1.5)
                frente()
                sleep(1.5)
                reTri()
                sleep(1.5)
                frente()
                sleep(1.5)
                reTri()

            frentee()

            motor_esq.run_forever(speed_sp=TP + control)
            motor_dir.run_forever(speed_sp=TP - control)

    except KeyboardInterrupt:
        motor_esq.stop()
        motor_dir.stop()

def giroDTri():
    motor_esq.run_to_rel_pos(position_sp=-400, speed_sp=200, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=400, speed_sp=200, stop_action="hold")
    sleep(0.5)


def voltar():
    client = mqtt.Client()
    Conectar(client)
    sleep(0.5)

    for i in range(0,25):
        frente()

        if (carga[1] >= 18 and carga[2] == 1):

            Sound.beep()
            stop()

            # Código para posicionar e derrubar as bolinhas no triângulo

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

            # código para retornar para a parede ou ir para o centro espalhar bolinhas:

            giroD()
            giroD()


        # Parando a conexão
    stop()
    Desconectar(client)



def frenteManual():

    #criando o client para se comunicar com o briak

    client = mqtt.Client()
    Conectar(client)
    sleep(0.5)

    while True:

        for i in range (0,30):
            frente()

            #se em algum momento ele ver o triângulo, ele deposita a bolinha e depois retorna para a parede

            if (carga[1] >= 18 and carga[2]==1):

                Sound.beep()
                stop()


                #Código para posicionar e derrubar as bolinhas no triângulo

                frente()
                frente()
                frente()
                sleep(0.5)
                descerr()
                sleep(0.5)
                vertiE()
                sleep(0.5)

                #parte que adicionei
                subirr()
                sleep(0.5)
                descerr()
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

                #código para retornar para a parede ou ir para o centro espalhar bolinhas:

                giroD()
                giroD()

                for i in range (0, 15):
                    frente()

        #poderia ter uma consição para caso ele visse o triângulo, quebrar o for e ele andar menos

        re()
        giroE()
        sleep(1.5)









    #Parando a conexão
    stop()
    Desconectar(client)


def sala3PID():

    descerr()

    while True:

        frentee()
        re()
        giroE()
        sleep(1.5)


def sala3Manual():

    descerr()

    while True:

        frenteManual()

def frentemenorrr():
    motor_esq.run_to_rel_pos(position_sp=60, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=60, speed_sp=400, stop_action="hold")
    sleep(0.5)

def reMaior():

    motor_esq.run_to_rel_pos(position_sp=-160, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-160, speed_sp=400, stop_action="hold")
    sleep(0.5)

def mcentro():
    for i in range (0,5):
        frente()
        sleep(0.5)
        frente()
        sleep(0.5)
        giroD()
        sleep(1.0)
        giroE()
        sleep(1.0)
        giroE()
        sleep(1.0)
        giroD()
        sleep(1.0)
        Sound.beep()
    for i in range(0,15):
        reMaior()

    frente()

def centro():

    for i in range (0,5):
        frente()
        sleep(0.5)
        frente()
        sleep(0.5)
        giroD()
        sleep(1.0)
        giroE()
        sleep(1.0)
        giroE()
        sleep(1.0)
        giroD()
        sleep(1.0)
        Sound.beep()

    #retornando para a parede ou triÂngulo

    for i in range(0,15):
        reMaior()

    frente()

    #girando para encontrar a parede ou triÂngulo




def frenteManual2():


    #criando o client para se comunicar com o brick

    client = mqtt.Client()
    Conectar(client)
    sleep(0.5)
    cont =0
    while True:
        for i in range (0,30):
            frente()

            #se em algum momento ele ver o triângulo, ele deposita a bolinha e depois retorna para a parede

            if (carga[1] >= 18 and carga[2]==1):
                cont = cont +1

                Sound.beep()
                stop()


                #Código para posicionar e derrubar as bolinhas no triângulo

                frente()
                sleep(0.5)
                descerr()
                sleep(0.5)
                vertiE()
                sleep(0.5)

                #parte que adicionei
                subirr()
                sleep(0.5)
                descerr()
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

                #código para ir até o centro e voltar
                if (cont == 2):
                    mcentro()
                    cont =0

                #código para retornar para a parede

                giroD()
                giroD()

                for i in range (0, 15):
                    frente()

        #poderia ter uma consição para caso ele visse o triângulo, quebrar o for e ele andar menos

        re()
        sleep(0.5)
        re()
        sleep(0.5)
        subirr()
        sleep(0.5)
        descerr()
        sleep(0.5)
        #se não der certo, acrescentar um frente equivalente ao ré
        frentemenorrr()
        sleep(0.5)
        giroE()
        sleep(1.5)


    #Parando a conexão
    stop()
    Desconectar(client)

def sala3Manual2():
    descerr()

    while True:
        frenteManual2()

def sala3():

    try:

        #1º etapa
        gfrente()
        frentee()
        re()
        giroE()
        sleep(1.5)
        frentinha()
        sleep(2.0)
        giroE()
        sleep(1.5)

        #2ºetapa, depois que girar totalmente

        frentee()
        sleep(1.5)
        re()
        re()
        giroD()
        sleep(1.5)
        frentinha()
        sleep(2.0)
        giroD()
        sleep(1.5)

        #3º etapa, depois de girar totalmente

        frentee()
        re()
        re()
        giroE()
        sleep(1.5)
        frentinha()
        sleep(2.0)
        giroE()
        sleep(1.5)

        #4º etapa, depois de girar totalmente

        frentee()
        re()
        re()
        giroD()
        sleep(1.5)
        frentinha()
        sleep(2.0)
        giroD()
        sleep(1.5)

        #5º etapa, depois de girar totalmente
        frentee()

        #if(ultra2.value()>)

        #etapa final
        reFinal()
        sleep(0.5)
        subir_descer()

        #procurar o triângulo

        mprocurar()






    except KeyboardInterrupt:
        motor_esq.stop()
        motor_dir.stop()

def ggfrente():
    client = mqtt.Client()
    Conectar(client)
    sleep(0.5)
    descerr()
    for i in range(0, 30):
        frente()
        if (carga[1] >= 10 and carga[1]<=40):
            Sound.beep()
            stop()
            sleep(0.5)
            frente()
            giroE()
            sleep((0.5))
            giroE()
            break

    stop()
    Desconectar(client)

def sala3ofi():
    client = mqtt.Client()
    Conectar(client)
    sleep(0.5)
    descerr()

    gfrente()
    re()
    giroE()
    sleep(1.5)
    frentinha()
    sleep(2.0)
    giroE()
    sleep(1.5)

    # 2ºetapa, depois que girar totalmente

    frentee()
    sleep(1.5)
    re()
    re()
    giroD()
    sleep(1.5)
    frentinha()
    sleep(2.0)
    giroD()
    sleep(1.5)

    # 3º etapa, depois de girar totalmente

    frentee()
    re()
    re()
    giroE()
    sleep(1.5)
    frentinha()
    sleep(2.0)
    giroE()
    sleep(1.5)

    # 4º etapa, depois de girar totalmente

    ggfrente()
    re()
    re()
    giroD()
    sleep(1.5)
    frentinha()
    sleep(2.0)
    giroD()
    sleep(1.5)

    # 5º etapa, depois de girar totalmente
    frentee()

    # if(ultra2.value()>)

    # etapa final
    reFinal()
    sleep(0.5)
    subir_descer()

    # procurar o triângulo

    mprocurar()

def reTri():
    motor_esq.run_to_rel_pos(position_sp=-60, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-60, speed_sp=400, stop_action="hold")
    sleep(0.5)

def subir_descer():
    subirr()
    descerr()
    subirr()
    descerr()
    subirr()

def reto():
    try:
        motor_esq.run_forever(speed_sp=TP )
        motor_dir.run_forever(speed_sp=TP )

    except KeyboardInterrupt:

        motor_dir.stop()
        motor_esq.stop()

def vertiE():
    motor_esq.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-460, speed_sp=400, stop_action="hold")
    sleep(0.5)

def vertiD():
    motor_esq.run_to_rel_pos(position_sp=-460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

def mre():
    motor_esq.run_to_rel_pos(position_sp=-700, speed_sp=1000, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-700, speed_sp=1000, stop_action="hold")
    sleep(0.5)

def mprocurar():
    client = mqtt.Client()
    Conectar(client)
    sleep(0.5)
    re()
    re()
    giroD()
    sleep(1.5)
    giroD()
    sleep(1.5)
    try:
        while True:
            valor = ultra1.value()
            print(valor)
            if(valor<= 80):
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

            motor_dir.run_forever(speed_sp=TP)
            motor_esq.run_forever(speed_sp=TP)
    except KeyboardInterrupt:
            stop()

sala3Manual2() #dando certo toda a varredura lateral

#centro()

#sala3ofi()
#reto()
#mprocurar()

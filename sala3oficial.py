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

TPDES=300
TPDES1=300

TPSUB=900
TPSUB1=80


# Motores
motor_dir = LargeMotor('outB')
motor_esq = LargeMotor('outD')
garra2 = MediumMotor('outA')

#sensores
ultra1 = UltrasonicSensor('in3')
ultra2= UltrasonicSensor('in4')

def subirr():
    garra2.run_timed(time_sp=1500, speed_sp=TPSUB)
    sleep(0.9)

def stop():
    motor_dir.stop()
    motor_esq.stop()

def frente():

    motor_esq.run_to_rel_pos(position_sp=160, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=160, speed_sp=400, stop_action="hold")
    sleep(0.3)

def descerr():

    garra2.run_timed(time_sp=1500, speed_sp=-TPDES)
    sleep(0.9)

def vertiE():
    motor_esq.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-460, speed_sp=400, stop_action="hold")
    sleep(0.5)

def vertiD():
    motor_esq.run_to_rel_pos(position_sp=-460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

def re():
    motor_esq.run_to_rel_pos(position_sp=-60, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-60, speed_sp=400, stop_action="hold")
    sleep(0.5)

def giroE():

    motor_esq.run_to_rel_pos(position_sp=500, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-500, speed_sp=400, stop_action="hold")
    sleep(0.5)

def giroD():
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
                giroTri()
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

def vertiE():
    motor_esq.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-460, speed_sp=400, stop_action="hold")
    sleep(0.5)

def mre():
    motor_esq.run_to_rel_pos(position_sp=-700, speed_sp=1000, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-700, speed_sp=1000, stop_action="hold")
    sleep(0.5)

def frenteRapido():
    motor_esq.run_to_rel_pos(position_sp=160, speed_sp=1000)
    motor_dir.run_to_rel_pos(position_sp=160, speed_sp=1000)
    sleep(0.3)

def coletaCentro():
    for i in range (0,15):
        frente()

    sleep(1.5)
    giroE()
    sleep(1.5)
    frente()
    sleep(0.5)
    frente()
    sleep(1.5)
    giroE()
    sleep(1.5)

    for i in range (0,25):
        frente()

    sleep(1.5)
    re()
    sleep(0.5)
    re()
    sleep(0.5)
    subirr()
    sleep(0.5)
    descerr()
    sleep(1.5)
    giroD()
    sleep(1.5)
    frente()
    sleep(0.5)
    frente()
    sleep(1.5)
    giroD()
    sleep(1.5)


    for i in range (0,25):
        frente()

    sleep(1.5)
    re()
    sleep(0.5)
    re()
    sleep(0.5)
    subirr()
    sleep(0.5)
    descerr()
    sleep(1.5)
    giroE()
    sleep(1.5)
    frente()
    sleep(0.5)
    frente()
    sleep(1.5)
    giroE()
    sleep(1.5)

    for i in range(0, 15):
        frente()

    sleep(1.5)
    re()
    sleep(0.5)
    re()
    sleep(0.5)
    subirr()
    sleep(0.5)
    descerr()
    sleep(1.5)
    giroD()
    sleep(1.5)
    frente()
    sleep(0.5)
    frente()
    sleep(1.5)
    giroD()
    sleep(1.5)

    for i in range(0, 10):
        frente()

    sleep(1.5)
    re()
    sleep(0.5)
    re()
    sleep(0.5)
    subirr()
    sleep(0.5)
    descerr()

    #depois que fizaer a varredura, retorna para a borda

    sleep(0.5)
    frente()
    sleep(0.5)
    giroD()
    sleep(1.5)
    giroD()
    sleep(1.5)

def coletaCentroRapido():

    for i in range (0,10):
        frenteRapido()

    sleep(1.5)
    giroE()
    sleep(1.5)
    frenteRapido()
    sleep(0.5)
    frenteRapido()
    sleep(1.5)
    giroE()
    sleep(1.5)

    for i in range (0,15):
        frenteRapido()

    sleep(1.5)
    re()
    sleep(0.5)
    re()
    sleep(0.5)
    subirr()
    sleep(0.5)
    descerr()
    sleep(1.5)
    giroD()
    sleep(1.5)
    frenteRapido()
    sleep(0.5)
    frenteRapido()
    sleep(1.5)
    giroD()
    sleep(1.5)


    for i in range (0,15):
        frenteRapido()

    sleep(1.5)
    re()
    sleep(0.5)
    re()
    sleep(0.5)
    subirr()
    sleep(0.5)
    descerr()
    sleep(1.5)
    giroE()
    sleep(1.5)
    frenteRapido()
    sleep(0.5)
    frenteRapido()
    sleep(1.5)
    giroE()
    sleep(1.5)

    for i in range(0, 12):
        frente()

    sleep(1.5)
    re()
    sleep(0.5)
    re()
    sleep(0.5)
    subirr()
    sleep(0.5)
    descerr()
    sleep(1.5)
    giroD()
    sleep(1.5)
    frenteRapido()
    sleep(0.5)
    frenteRapido()
    sleep(1.5)
    giroD()
    sleep(1.5)

    for i in range(0, 8):
        frenteRapido()

    sleep(1.5)
    re()
    sleep(0.5)
    re()
    sleep(0.5)
    subirr()
    sleep(0.5)
    descerr()

    #depois que fizaer a varredura, retorna para a borda

    sleep(0.5)
    frenteRapido()
    sleep(0.5)
    giroD()
    sleep(1.5)
    giroD()
    sleep(1.5)

def mrefinal():
    motor_esq.run_to_rel_pos(position_sp=-800, speed_sp=1000, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-800, speed_sp=1000, stop_action="hold")
    sleep(0.5)

def frenteManualMeio():
    #criando o client para se comunicar com o brick

    client = mqtt.Client()
    Conectar(client)
    sleep(0.5)
    cont =0

    #faz uma varredura manual no centro:

    coletaCentro()

    #faz a varredura na lateral e vai no centro

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
                    #mcentro()

                    ##### colocar codigo para ele procurar a saída e varrer o centro novamente
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

def andarParaentrada():

    giroD()
    giroD()
    sleep(1.5)

    client = mqtt.Client()
    Conectar(client)
    sleep(0.5)



    while(carga[1]<=60):

        for i in range (0,25):
            frente()

        sleep(0.5)
        re()
        sleep(1.5)
        giroE()



    sleep(0.5)
    re()
    sleep(0.5)
    re()
    sleep(0.5)
    giroE()
    sleep(1.5)
    giroE()
    sleep(0.5)

    stop()
    Desconectar(client)

def frenteManualBorda():
    # criando o client para se comunicar com o brick

    client = mqtt.Client()
    Conectar(client)
    sleep(0.5)
    cont = 0
    while True:
        for i in range(0, 30):
            frente()

            # se em algum momento ele ver o triângulo, ele deposita a bolinha e depois retorna para a parede

            if (carga[1] >= 18 and carga[2] == 1):
                cont = cont + 1

                Sound.beep()
                stop()
                # Código para posicionar e derrubar as bolinhas no triângulo

                frente()
                sleep(0.5)
                descerr()
                sleep(0.5)
                vertiE()
                sleep(0.5)

                # parte que adicionei
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
                mrefinal()
                sleep(0.5)
                frente()
                stop()

                # código para ir até o centro e voltar
                if (cont == 2):
                    andarParaentrada()
                    coletaCentro()
                    cont = 0

                # código para retornar para a parede

                giroD()
                giroD()

                for i in range(0, 15):
                    frente()

        # poderia ter uma consição para caso ele visse o triângulo, quebrar o for e ele andar menos

        re()
        sleep(0.5)
        re()
        sleep(0.5)
        subirr()
        sleep(0.5)
        descerr()
        sleep(0.5)
        # se não der certo, acrescentar um frente equivalente ao ré
        frentemenorrr()
        sleep(0.5)
        giroE()
        sleep(1.5)

    # Parando a conexão
    stop()
    Desconectar(client)

def frenteManualMeioRapido():

    descerr()

    client = mqtt.Client()
    Conectar(client)
    sleep(0.5)
    cont = 0

    # faz uma varredura manual no centro:

    coletaCentroRapido()

    # faz a varredura na lateral e vai no centro

    while True:
        for i in range(0, 20):
            frenteRapido()

            # se em algum momento ele ver o triângulo, ele deposita a bolinha e depois retorna para a parede

            if (carga[1] >= 18 and carga[2] == 1):
                cont = cont + 1

                Sound.beep()
                stop()

                # Código para posicionar e derrubar as bolinhas no triângulo

                frenteRapido()
                sleep(0.5)
                frenteRapido()
                sleep(0.5)
                descerr()
                sleep(0.5)
                vertiE()
                sleep(0.5)

                # parte que adicionei
                subirr()
                sleep(0.5)
                descerr()
                sleep(0.5)

                mre()
                sleep(0.5)
                frenteRapido()
                sleep(0.5)
                mre()
                sleep(0.5)
                frenteRapido()
                sleep(0.5)
                mre()
                sleep(0.5)
                frenteRapido()
                stop()

                # código para ir até o centro e voltar
                if (cont == 2):
                    # mcentro()

                    ##### colocar codigo para ele procurar a saída e varrer o centro novamente
                    cont = 0

                # código para retornar para a parede

                giroD()
                giroD()

                for i in range(0, 10):
                    frenteRapido()

        # poderia ter uma consição para caso ele visse o triângulo, quebrar o for e ele andar menos

        re()
        sleep(0.5)
        re()
        sleep(0.5)
        subirr()
        sleep(0.5)
        descerr()
        sleep(0.5)
        # se não der certo, acrescentar um frente equivalente ao ré
        frentemenorrr()
        sleep(0.5)
        giroE()
        sleep(1.5)

    # Parando a conexão
    stop()
    Desconectar(client)

def sala3ManualMeio():
    # varrendo primeiro no meio e depois nas laterais

    descerr()

    while True:
        frenteManualMeio()

def sala3ManualBorda():

    descerr()

    while True:

        frenteManualBorda()

def sala3ManualMeioRapido():
    descerr()

    while True:
        frenteManualMeioRapido()


#coletaCentroRapido()

sala3ManualMeioRapido() #dandooooooooooooooooooo certoooooooooooooooooooooooo
#sala3ManualBorda()
#sala3ManualMeio()
#sala3Manual2() #dando certo toda a varredura lateral
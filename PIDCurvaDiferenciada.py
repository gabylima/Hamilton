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
carga =" "
TPDES=900
cont_rampa = 0
ults3=0

sensor_cor=0
TPSUB=800
TPS3 = 700
TP_procurar = 400

cont_s3 = -1
contador_lado = 0
contLateral = 0

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


def subirr():
    garra2.run_timed(time_sp=1500, speed_sp=TPSUB)
    sleep(0.9)

def mprocurar():
    client = mqtt.Client()
    Conectar(client)
    sleep(0.5)
    subirr()
    sleep(0.5)
    giroD()
    sleep(1.5)
    try:
        while True:
            valor = ultra1.value()
            if(valor<=50):
                re()
                re()
                vertiE()
                sleep(0.5)
                stop()
                if(carga[1]>=18 ) :
                   frente()
                   frente()
                   frente()
                   sleep(0.5)
                   descerr()
                   sleep(0.5)
                   vertiE()
                   sleep(0.5)
                   re_triangulo()
                   sleep(0.5)
                   frente()
                   sleep(0.5)
                   re_triangulo()
                   sleep(0.5)
                   frente()
                   sleep(0.5)
                   re_triangulo()
                   sleep(0.5)
                   frente()
                   stop()
                   break

                else:
                    if(valor <= 80 and carga[1] <= 20):
                        stop()
                        re()
                    elif(valor <= 80 and carga[1]>=40):
                        stop()
                        re()

            motor_dir.run_forever(speed_sp=TPS3)
            motor_esq.run_forever(speed_sp=TPS3)
    except KeyboardInterrupt:
            stop()

def vertiE():
    motor_esq.run_to_rel_pos(position_sp=-460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    sleep(0.5)

def descerr():

    garra2.run_timed(time_sp=1500, speed_sp=-TPDES)
    sleep(0.9)

def vertiD():
    motor_esq.run_to_rel_pos(position_sp=460, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-460, speed_sp=400, stop_action="hold")
    sleep(0.5)

def re_triangulo():
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
            global cont_s3
            cont_s3 = cont_s3 + 1
            break
        tempo = tempo + timedelta(seconds=1)
        sleep(1)

def restartsala3():
    button = Button()
    if button.left:
        while True:
            stop()
            if button.right:
                global sair
                sair = True
                break

def Principal():
    global contador_lado

    while True:

        motor_dir.run_forever(speed_sp=TPS3)
        motor_esq.run_forever(speed_sp=TPS3)

        if contador_lado == 2:
            stop()
            Sound.beep()
            break

        if cont_s3 == -1:
            CriarCronometro('0:00:02')
        else:
            CriarCronometro('0:00:04')

        if cont_s3 % 2 == 0:
            TurnoE()
            contador_lado = contador_lado + 1
            subirr()
            sleep(0.5)
            descerr()
        else:
            TurnoD()
            contador_lado = contador_lado + 1
            subirr()
            sleep(0.5)
            descerr()

def giroE():

    motor_esq.run_to_rel_pos(position_sp=-480, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=480, speed_sp=400, stop_action="hold")
    sleep(0.5)

def giroD():
    motor_esq.run_to_rel_pos(position_sp=480, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-480, speed_sp=400, stop_action="hold")
    sleep(0.5)

def TurnoE():
    stop()
    re()
    re()
    sleep(1.4)
    giroE()
    sleep(1.4)
    frente()
    frente()
    frente()
    frente()
    sleep(1.4)
    giroE()
    sleep(1.4)

def TurnoD():
    stop()
    re()
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
            contLateral=contLateral+1
            break

        tempo = tempo + timedelta(seconds=1)
        sleep(1)

def reMaior():
    motor_dir.run_to_rel_pos(position_sp=-100, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-100, speed_sp=400, stop_action="hold")
    sleep(0.5)
    motor_dir.run_to_rel_pos(position_sp=-100, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-100, speed_sp=400, stop_action="hold")
    sleep(0.5)
    motor_dir.run_to_rel_pos(position_sp=-100, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-100, speed_sp=400, stop_action="hold")
    sleep(0.5)
    motor_dir.run_to_rel_pos(position_sp=-100, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-100, speed_sp=400, stop_action="hold")
    sleep(0.5)

def sala3():
    descerr()

    Principal() #varre no meio

    VarreduraLateral() #varre nas bordas e procura o triângulo

    mprocurar()
    #depois varrer mais uma vez na lateral e depositar

def VarreduraLateralkk():
    descerr()
    sleep(0.5)
    frente()
    sleep(1.5)
    giroE()
    while True:
        motor_esq.run_forever(speed_sp =TPS3 )
        motor_dir.run_forever(speed_sp =TPS3 )
        CriarCronometro("0:00:03")
        stop()
        re()
        re()
        sleep(1.5)
        giroE()
        break

def VarreduraLateral():
    descerr()
    try:
        while contLateral<= 4:
            motor_esq.run_forever(speed_sp=TPS3)
            motor_dir.run_forever(speed_sp=TPS3)
            CronometroLateral2('0:00:05')
            stop()
            re()
            subirr()
            sleep(0.5)
            descerr()
            sleep(0.5)
    except KeyboardInterrupt:
        stop()

def giro(SP):
    mfrentemenor()
    pid = PID(KP, KI, KD, setpoint=SP)
    while True:
        diferenca = sensor_esq.value() - sensor_dir.value()
        control = pid(diferenca)
        if (diferenca == 0):
            stop()
            break
        print(diferenca)
        motor_esq.run_forever(speed_sp = -control)
        motor_dir.run_forever(speed_sp = control)

def rampa(SP):

    KPR = 10 # modifiquei o kp: tava 12
    KIR = 0
    KDR = 0
    TPR = 1000.0  # modifiquei tp: tava 280

    V_MAX_MOTOR = 1000
    VALOR_MAX_CONTROL = V_MAX_MOTOR - TPR

    pid = PID(KPR, KIR, KDR, setpoint=SP)
    #print(TPR, KPR)7

    while True:


        valor = sensor_esq.value() - sensor_dir.value()
        control = pid(valor)
        if (control > VALOR_MAX_CONTROL):
             control = VALOR_MAX_CONTROL
        elif (control < -VALOR_MAX_CONTROL):
             control = -VALOR_MAX_CONTROL


        motor_esq.run_forever(speed_sp=TPR - control)
        motor_dir.run_forever(speed_sp=TPR + control)


def Restart():
    button = Button()
    if button.left:
        while True:
            stop()
            if button.right:
                break

def VerificaSala3(c,SP):
    if c >= 9:

      rampa(SP)

    if (ultra2.value() >= 300 and carga[1] <= 4 and c >= 9):
        return 1
    elif ((ultra2.value() >= 30 and ultra2.value() <= 78) and carga[1] >= 70 and c >= 9):
        return 1
    elif (ultra2.value() >= 300 and carga[1] >= 70):
        global cont_rampa
        cont_rampa = 0

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
    ReObstaculo()
    stop()

def Conectar(client):
    client.connect("192.168.2.43", 1883, 60)
    # client.connect("10.42.0.183", 1883, 60)
    client.on_connect = on_connect
    client.on_message = on_message
    client.loop_start()

def Desconectar(client):
    client.loop_stop()
    client.disconnect()

def mfrentemenor():
    #volta so um pouco para tras
    motor_dir.run_to_rel_pos(position_sp=80, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=80, speed_sp=400, stop_action="hold")
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

    motor_dir.run_to_rel_pos(position_sp=-80, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-80, speed_sp=400, stop_action="hold")
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

def DualGreen():

    esquerdo = carga[2]
    direito = carga[3]

    if(esquerdo == 3 and direito == 3):
        twoverde()
        mfrentemenor()

    if(esquerdo == 3 or direito == 3):
        #giro(TP)
        mtras()
        if(direito == 1 and esquerdo == 1):
            mfrente()
        elif(direito == 1 and esquerdo== 6):
            mfrente()
        elif (direito == 6 and esquerdo == 1):
            mfrente()
        elif(direito == 6 and esquerdo == 6):
            mfrentemenor()
            if(direito==3 and (esquerdo==6 or esquerdo==1)):
                DgiroVerde()
                mfrentemenor()
            elif(esquerdo == 3 and (direito==6 or direito==1)):
                EgiroVerde()
                mfrentemenor()

def ReObstaculoTras():
    motor_dir.run_to_rel_pos(position_sp=-40, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-40, speed_sp=400 + CORRECAO_MOTOR, stop_action="hold")
    sleep(0.5)

def trasVerde():
    motor_dir.run_to_rel_pos(position_sp=-60, speed_sp=400, stop_action="hold")
    motor_esq.run_to_rel_pos(position_sp=-60, speed_sp=400, stop_action="hold")
    sleep(0.5)

def verificarVerde2():
    stop()
    sensor_esq.mode = 'COL-COLOR'
    sensor_dir.mode = 'COL-COLOR'

    verdeE = sensor_esq.value()
    verdeD = sensor_dir.value()
    Sound.beep()
    if (verdeD == 3 and verdeE == 3):
        twoverde()

    elif (verdeE==3 or verdeD==3):

        trasVerde()
        stop()

        verdeE = sensor_esq.value()
        verdeD = sensor_dir.value()

        if (verdeD == 6 and verdeE == 1):
            frente()
            frente()

        elif (verdeD == 1 and verdeE == 6):
            frente()
            frente()
        elif (verdeD == 6 and verdeE == 6):
            mfrentemenor()
            stop()
            verdeE = sensor_esq.value()
            verdeD = sensor_dir.value()

            if (verdeD == 3):
                frente()
                DgiroVerde()
                Sound.beep()
            elif (verdeE == 3):
                frente()
                EgiroVerde()
                Sound.beep()
            elif (verdeD == 3 and verdeE ==3):
                twoverde()

    sensor_esq.mode = 'COL-REFLECT'
    sensor_dir.mode = 'COL-REFLECT'

def verificarVerde():
    stop()
    sensor_esq.mode = 'COL-COLOR'
    sensor_dir.mode = 'COL-COLOR'

    verdeE = sensor_esq.value()
    verdeD = sensor_dir.value()
    Sound.beep()
    if(verdeD == 3 and verdeE == 3):
        twoverde()
    else:
        mtras()
        stop()

        verdeE = sensor_esq.value()
        verdeD = sensor_dir.value()

        if( verdeD == 6 and verdeE == 1):
            frente()
        elif(verdeD == 1 and verdeE == 6):
            frente()
        elif(verdeD == 6 and verdeE == 6):
            mfrentemenor()
            stop()
            verdeE = sensor_esq.value()
            verdeD = sensor_dir.value()

            if(verdeD == 3):
                frente()
                DgiroVerde()
                Sound.beep()
            elif(verdeE == 3):
                frente()
                EgiroVerde()
                Sound.beep()

    sensor_esq.mode = 'COL-REFLECT'
    sensor_dir.mode = 'COL-REFLECT'

def TurnoEkk():
    stop()
    re()
    re()
    sleep(1.4)
    giroEkk()
    sleep(1.4)
    frente()
    frente()
    frente()
    frente()
    sleep(1.4)
    giroEkk()
    sleep(1.4)

def TurnoDkk():
    stop()
    re()
    re()
    sleep(1.4)
    giroDkk()
    sleep(1.4)
    frente()
    frente()
    frente()
    sleep(1.4)
    giroDkk()
    sleep(1.4)

def giroDkk():
    motor_esq.run_to_rel_pos(position_sp=500, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-500, speed_sp=400, stop_action="hold")
    sleep(0.5)

def Principalkk():
    global contador_lado

    while True:

        motor_dir.run_forever(speed_sp=TPS3)
        motor_esq.run_forever(speed_sp=TPS3)

        if contador_lado == 2:
            stop()
            Sound.beep()
            break

        if cont_s3 == -1:
            CriarCronometro('0:00:02')
        else:
            CriarCronometro('0:00:04')

        if cont_s3 % 2 == 0:
            TurnoE()
            contador_lado = contador_lado + 1
            subirr()
            sleep(0.5)
            descerr()
        else:
            TurnoD()
            contador_lado = contador_lado + 1
            subirr()
            sleep(0.5)
            descerr()
    ajuste()


    while True:
        motor_dir.run_forever(speed_sp=TPS3)
        motor_esq.run_forever(speed_sp=TPS3)

        CriarCronometro('0:00:06')

        re()
        re()
        re()
        subirr()
        sleep(0.5)
        descerr()
        sleep(0.5)
        subirr()
        break

def giroEkk():

    motor_esq.run_to_rel_pos(position_sp=-500, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=500, speed_sp=400, stop_action="hold")
    sleep(0.5)

def VarreduraM():
    try:
        descerr()
        while contLateral <= 10:
            motor_dir.run_forever(speed_sp=TPS3)
            motor_esq.run_forever(speed_sp=TPS3)
            CronometroLateral2("0:00:03")
            re()
            re()
            vertical()
            sleep(0.5)
            stop()
            subirr()
            descerr()
    except KeyboardInterrupt:
        motor_dir.stop()
        motor_esq.stop()

def vertical():
    motor_esq.run_to_rel_pos(position_sp=-400, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=400, speed_sp=400, stop_action="hold")
    sleep(0.5)

def ajuste():
    motor_esq.run_to_rel_pos(position_sp=120, speed_sp=400, stop_action="hold")
    motor_dir.run_to_rel_pos(position_sp=-120, speed_sp=400, stop_action="hold")
    sleep(0.5)

def varreduraUltra():
    client = mqtt.Client()
    Conectar(client)
    sleep(0.5)

    ultras3 = carga[1]

    while carga[1] >=12:

        giroE()

def sala3kk():
    descerr()
    Principalkk()
    VarreduraM()
    mprocurar()

def VeriVerde():
    sensor_esq.mode = 'COL-COLOR'
    sensor_dir.mode = 'COL-COLOR'

    verdeE = sensor_esq.value()
    verdeD = sensor_dir.value()

    if ((verdeE == 3) and (verdeD == 3)):
        # condição para verificar se há dois verdes

        twoverde()
    elif (verdeE == 3):  # condição caso ele veja verde apenas com o sensor esquerdo

        # Primeiro anda um pouco para trás para verifica se a linha de tras é branca ou preta

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

def executar(SP):
    TP = 250.0  # modifiquei tp: tava 280
    pid = PID(KP, KI, KD, setpoint=SP)
    lista = []
    client = mqtt.Client()
    Conectar(client)
    sleep(0.5)
    try:
        while True:
            Restart()

            #parte da rampa

            if((ultra2.value()>=36 and ultra2.value()<=80)and(carga[1]<=2)):
                global cont_rampa
                cont_rampa= cont_rampa + 1
            s3 = VerificaSala3(cont_rampa, SP)

        #parte do obstaculo

            if ultra1.value() <= 70:
                stop()
                Sound.beep()
                ReObstaculoTras()
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
            if (dicionario["esquerdo"]- 2 < sensor_dir.value() < dicionario["esquerdo"]+ 2):
                VeriVerde()

            elif (dicionario["direito"]- 2 < sensor_esq.value() < dicionario["direito"] + 2):
                VeriVerde()

        # condições para evitar ultrapassar o valor máximo do motor
            if (control > VALOR_MAX_CONTROL):
                control = VALOR_MAX_CONTROL
            elif (control < -VALOR_MAX_CONTROL) :
                control= -VALOR_MAX_CONTROL

        #motores com o PID
            motor_esq.run_forever(speed_sp=TP - control + CORRECAO_MOTOR)
            motor_dir.run_forever(speed_sp=TP + control)

            if(s3==1):
                stop()
                sala3kk()
                global cont_rampa
                cont_rampa =0


    except KeyboardInterrupt:

        motor_dir.stop()
        motor_esq.stop()
        print('cont:', cont_rampa)

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
            SP = calibragem(button2)
            print('SP atualizado:',SP)
            sleep(0.01)
            system("clear")
            sleep(0.01)
            print("<< Calibrar | Iniciar >>")


        elif button2.right:
            system("clear")
            executar(SP)
            break

def kk ():

    while True:
        sensor_dir.mode = 'COL-COLOR'
        sensor_esq.mode = 'COL-COLOR'
        print("sensor direito: ",sensor_dir.value())
        print("sensor esquerdo: ",sensor_esq.value())


#mprocurar()
menu()
#sala3kk()





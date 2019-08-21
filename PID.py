#!/usr/bin/env python3
from datetime import  timedelta
from ev3dev.ev3 import *
from time import sleep

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
    print(tempo)
    return True


t = time.gmtime()

print(t.tm_sec)
while t.tm_sec == 45:


#!/usr/bin/env python3
from ev3dev.ev3 import *
from ev3dev2.motor import *
import os
from time import *



# def gyroCal():
#     port = LegoPort("in1")
#     port.mode = "other-uart"
#     sleep(1)
#     port.mode = "auto"
#     sleep(5)
    



# gyroCal()
# gy = GyroSensor()
# while True:
#     angle = gy.angle
#     print(str(angle) + ' degrees')

mot = MediumMotor(OUTPUT_B)
mot.on(-25)
sleep(1.5)
# mL = LargeMotor('outB')

# mL.on_for_degrees(25,180)
# sleep(5)
# mL.on_for_degrees(25,-180)
# sleep(5)
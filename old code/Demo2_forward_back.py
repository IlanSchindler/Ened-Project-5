#!/usr/bin/env python3
from ev3dev.ev3 import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor import *
from ev3dev2.motor import *
#from ev3dev2.port import *
import os
from time import *

def GyroCalibration(port):
    LegoPort(port).mode = "other-uart"
    sleep(2)
    LegoPort(port).mode = "ev3-uart"
    sleep(2)


stepList = [60, 75, 105, 90, 60]

GyroCalibration('in2')
sleep(2)
gy = GyroSensor(INPUT_2)
gy.mode = "GYRO-RATE"
gy.mode = "GYRO-ANG"

steerPair = MoveSteering(OUTPUT_B, OUTPUT_C)

cmPerSecForward = 26.54
cmPerSecBack = 27.42

motorPower = 5

forceT = 10
for i in range(len(stepList)):
    if i % 2 == 0:
        t = stepList[i] / cmPerSecForward
        
        startTime = time()
        while time() - startTime < t:
            angle1 = gy.angle * -1
            # if angle1 >= 0:
            #     angle = (((-1 * angle1) % 360) * 100 / 360) * -1
            # else:
            #     angle = (angle1 % 360) * 100 / 360
            steerPair.on(angle1, motorPower)
            # print(str(round(angle1,0)), str(round(angle,0)))
        steerPair.off(brake = True)
    else:
        t = stepList[i] / cmPerSecBack
        startTime = time()
        while time() - startTime < t:
            angle1 = gy.angle * -1
            # if angle1 >= 0:
            #     angle = (((-1 * angle1) % 360) * 100 / 360) * -1
            # else:
            #     angle = (angle1 % 360) * 100 / 360
            steerPair.on(angle1 * -1, motorPower * -1)
            # print(str(round(angle1,0)), str(round(angle,0)))
        steerPair.off(brake = True)
    wait_for_bump('enter')

for j in range(len(stepList)):
    i = j + 1
    iFind = len(stepList) - i
    if i % 2 == 0:
        t = stepList[iFind] / cmPerSecForward
        
        startTime = time()
        while time() - startTime < t:
            angle1 = gy.angle * -1
            # if angle1 >= 0:
            #     angle = (((-1 * angle1) % 360) * 100 / 360) * -1
            # else:
            #     angle = (angle1 % 360) * 100 / 360
            steerPair.on(angle1, motorPower)
            # print(str(round(angle1,0)), str(round(angle,0)))
        steerPair.off(brake = True)
    else:
        t = stepList[iFind] / cmPerSecBack
        startTime = time()
        while time() - startTime < t:
            angle1 = gy.angle * -1
            # if angle1 >= 0:
            #     angle = (((-1 * angle1) % 360) * 100 / 360) * -1
            # else:
            #     angle = (angle1 % 360) * 100 / 360
            steerPair.on(angle1 * -1, motorPower * -1)
            # print(str(round(angle1,0)), str(round(angle,0)))
        steerPair.off(brake = True)
    wait_for_bump('enter')

# os.system('setfont Lat15-TerminusBold14')
# mL = LargeMotor('outB'); mL.stop_action = 'hold'
# mR = LargeMotor('outC'); mR.stop_action = 'hold'
# print('Hello, my name is EV3!')
# Sound.speak('Hello, my name is EV3!').wait()
# mL.run_to_rel_pos(position_sp= 840, speed_sp = 250)
# mR.run_to_rel_pos(position_sp=-840, speed_sp = 250)
# mL.wait_while('running')
# mR.wait_while('running')
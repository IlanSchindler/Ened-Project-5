#!/usr/bin/env python3
from ev3dev.ev3 import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor import *
from ev3dev2.motor import *
from ev3dev2.button import *
from array import *
from math import *
#from ev3dev2.port import *
import os
from time import *

#some global variables

os.system('setfont Lat15-TerminusBold14') #sets the font on the screen for easier reading
debug = False #determines if the robot is in debug mode
scaleDebug = False #determines if the robot is in scaled-debug mode, for if the testing surface is small
btn = Button() #the buttons on the robot


stepList = [30, 60, 90, 60, 60] #the list of steps the robot makes

motorPower = 30 #the power the motors will have when moving
lowMotorPower = 1

wheelCirc = 6.88 * pi #the circumference of the wheels, used to calculate the travel distance of the robot

#gyro initialization - initializing the gyro sensor
#sometimes, the program thinks that there isn't a gyro sensor plugged in when there actually is,
#so when the gyrosensor is initialized, it throws an error. This loop will repeatedly try to intialize
#the gyro sensor until it succeeds
gy = None
while(gy is None):
    try:
        LegoPort('in1').mode = "other-uart"
        sleep(2)
        LegoPort('in1').mode = "ev3-uart"
        sleep(4)

        gy = GyroSensor()
        gy.mode = "GYRO-RATE"
        gy.mode = "GYRO-ANG"
    except:
        gy = None
        continue

#this resets the current angle of the gyro sensor to 0
def GyroReset():
    gy.mode = "GYRO-RATE"
    gy.mode = "GYRO-ANG"
    sleep(1)

#motor variable declarations

#motor pair for steering
steerPair = MoveSteering(OUTPUT_B, OUTPUT_C)
#individual motors
leftMotor = LargeMotor(OUTPUT_B)
rightMotor = LargeMotor(OUTPUT_C)

#this calculated the average angle that both motors have rotated
def steerPos():
    leftRot = leftMotor.position
    rightRot = rightMotor.position
    return (leftRot + rightRot) / 2

#this resents the current angle that both motors have rotated to 0
def steerReset():
    leftMotor.position = 0
    rightMotor.position = 0

#this method is used when the robot is in debug mode. 
#it will print that it is waiting, and how many times it has waited during this run of the program
#the program then waits for the center, or 'enter' button on the robot to be pressed before continueing
i = 0
def debugWait():
    global i
    i = i + 1
    print("Waiting: ", str(i))    
    btn.wait_for_bump('enter')
    sleep(.1)

#this method inputs if the robot is moving forward or backward, then calculated and returns the steering value
#that will adjust the robot to keep driving straight
def getAngle(direction):
    angle = gy.angle
    angle1 = (((abs(angle) % 360)) * 100 / 180) * direction
    if(angle < 0):
        return angle1
    else:
        return angle1 * -1

#this moves the robot the given distance in the given direction.
#to know how far the robot has moved, the number of degrees the motors have rotated is used along with the circumference of the wheels
#the robot will move until it has overshot its target distance, then will move the opposite direction slowly until it hits its
# mark more precisely
def moveDistance(distance, direction):
    distanceAngle = distance * 360 / wheelCirc
    steerReset()

    while abs(steerPos()) < distanceAngle:
        steerPair.on(getAngle(direction), motorPower * direction)
    steerPair.off(brake = True)

    remainingAngle = 2
    while (abs(int(remainingAngle)) > 0):
        remainingAngle = distanceAngle - abs(steerPos())
        steerPair.on_for_degrees(getAngle(direction), lowMotorPower * direction, remainingAngle)
    steerPair.off(brake = True)
    sleep(.5)

    #if the program is in debug mode, the robot will wait until the center button is pressed before continueing
    if(debug):
        debugWait()

#this method scales down the distances by a given factor if the program is being debugged
#this is useful for if the surface being used for testing is very small
scaleFactor = 1
def debugScale():
    for i in range(len(stepList)):
        stepList[i] = stepList[i] / scaleFactor

#main code
#this loop steps trough every distance in the stepList list, twice, then calls moveDistance to move the robot that distance.
#because the robot always moves in the opposite direction to what it just did, after moving, the direction is multiplied by -1
#to reverse the direction of the next movement.
#the first loop through stepList travels the distances in order from the first index to the last, the second loop
#goes from the last to the first

def main():
    if scaleDebug:
        debugScale()
    direction = 1
    for i in range(len(stepList)):    
        moveDistance(stepList[i], direction)
        direction = direction * -1
    for i in range(len(stepList)):    
        moveDistance(stepList[len(stepList) - (i + 1)], direction)
        direction = direction * -1

print("Ready To Start")
btn.wait_for_bump('enter')
main()
        

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
btn = Button() #the buttons on the robot

#the set of coordinates the robot must goto
cords = [[0,   0, -73, -73, 0],\
         [0, -40, -40, -73, 0]]

waitTimeAtEnd = 20 #the time the robot must wait at its destination point

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
#motor pair for Tank
tankPair = MoveTank(OUTPUT_B, OUTPUT_C)
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

#this method determines which direction the robot needs to face to move to its new coordinate, then returns that angle
# north = 0   (no change in x, increase in y)
# east = 90   (increase in x,  no change in y)
# south = 180 (no change in x, decrease in y)
# west = 270  (decrease in x,  no change in y)
#if the robot doesn't move, it returns north
def getDirection(oldX, oldY, newX, newY):
    if(oldX == newX):
        if(oldY < newY):
            return 0
        elif (oldY > newY): #oldY > newY
            return 180
        else: #oldX = newX, oldY = newY -> dont move
            return 0
    else: #oldX != newX, oldY == newY
        if(oldX < newX):
            return 90
        else: #oldX > newX
            return 270

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


def calculateAngleProper(inAngle):
    angle = degrees(acos(cos(radians(inAngle))))
    if(inAngle % 360) > 180:
        angle = angle * -1
    return round(angle, 0)


#this method will rotate the robot to face (newX, newY) from its current position (oldX, oldY), given that one 
#of the coordinates is the same. The robot will only spin clockwise
def rotateToDirection(oldX, oldY, newX, newY, currentDirection):
    #calculates how far the robot needs to turn
    direction = getDirection(oldX, oldY, newX, newY)
    delta = (direction - currentDirection) % 360
    delta = calculateAngleProper(delta)
    spinDirection = 0
    if delta == 0:
        spinDirection = 1
    else:
        spinDirection = delta / abs(delta)

    #begins turning robot clockwise until it has turned delta degrees
    tankPair.on((motorPower * spinDirection), (motorPower * -1 * spinDirection))
    gy.wait_until_angle_changed_by(delta, direction_sensitive = True)
    tankPair.off(brake=True)
    sleep(.25)

    #the robot usually overshoots the turn, so this rotates the robot counter-clockwise much slower than clockwise.
    #this is done until it reaches the angle it was supposed to turn to originally.
    tankPair.on(lowMotorPower * -1 * spinDirection, lowMotorPower * spinDirection)
    if abs(delta) == 180:
        while (gy.angle - currentDirection) % 360 > abs(delta):
            pass
    else: 
        while abs(calculateAngleProper(gy.angle - currentDirection)) > abs(delta):
            pass
    tankPair.off(brake = True)
    sleep(.25)

    #if the program is in debug mode, the robot will wait for the center button to be pressed before continuing
    if(debug):
        debugWait()

    return direction

#the method inputs the direction the robot should currently be facing
#   0 = N, 90 = E, 180 = S, 270 = W
#and measures the current reading on the gyro sensor. then it calculates what the steering value needs
#to be for the robot to adjust to any drifting while driving
#if the robot veers slightly left of its intended path, this will return a steering value to turn it slightly right
def getAngle(direction):
    angle = gy.angle
    angle = (angle - direction) % 360
    if(angle > 180):
        return ((abs(angle - 360)) * 100 / 180)
    else:
        return (angle * 100 / 180) * -1

#moves robot from (oldX, oldY) to (newX, newY) given that either the x or y coordinate doesn't change
#if both do change, an if block in the main code will ensure this is called twice,
#   once to move along X, then once to move along Y
def moveCord(oldX, oldY, newX, newY, currentDirection):
    #call to rotateToDirection to properly orient the robot before moving forward
    direction = rotateToDirection(oldX, oldY, newX, newY, currentDirection)

    #calculates the distance the robot needs to travel in terms of degrees the motors need to rotate
    distance = abs((newX - oldX) + (newY - oldY))
    distanceAngle = distance * 360 / wheelCirc
    steerReset()

    #moves the robot forward until the motors have rotated the previously calculate number of degrees
    #gyro sensor is continually checked to ensure a straight path
    while abs(steerPos()) < distanceAngle:
        steerPair.on(getAngle(direction), motorPower)
    steerPair.off(brake = True)

    #due to inertia, the robot tends to overshoot the distance. this block moves the robot either 
    #forward (undershoot) or backward (overshoot) at a much slower speed to adjust for this
    remainingAngle = 2
    while (abs(remainingAngle) > 1):
        remainingAngle = distanceAngle - steerPos()
        steerPair.on_for_degrees(getAngle(direction), lowMotorPower, remainingAngle)
    steerPair.off(brake = True)
    sleep(.5)

    #if the program is in debug mode, the robot will wait until the center button is pressed before continueing
    if(debug):
        debugWait()

    return direction

#main code
#a for loop goes through every set of coordinates in the cords array and gets the current (x,y) and next (x,y) of the robot.
#because the robot can only move directly along the x or y axis, for every move, either oldX = newX OR oldY = newY.
#if that is not the case, and the robot must move to another point that isnt orthagonally related to it, the robot will
#move twice, once along the x, then once along the y. if this measure were not taken, it is unclear what the robot would do.
#if the next coordinate it is going to is (0,0), then the robot has reached the desired final point and will reorient itself
#to face +y, then wait the given amound of time, before moving to (0,0)
#after it has gone through all the points, it will reorient itself to face +y
def main():
    currentDirection = 0
    for j in range(len(cords[0]) - 1):
        oldX = cords[0][j]
        oldY = cords[1][j]
        newX = cords[0][j + 1]
        newY = cords[1][j + 1]
        if(newX == 0 and newY == 0):
            currentDirection = rotateToDirection(oldX, oldY, oldX, oldY, currentDirection)
            sleep(waitTimeAtEnd)    
        if(oldX != newX and oldY != newY):
            currentDirection = moveCord(oldX, oldY, oldX, newY, currentDirection)
            currentDirection = moveCord(oldX, newY, newX, newY, currentDirection)
        else:
            currentDirection = moveCord(oldX, oldY, newX, newY, currentDirection)
    currentDirection = rotateToDirection(0, 0, 0, 0, currentDirection)        


print("Ready to Start")
btn.wait_for_bump('enter')
main()

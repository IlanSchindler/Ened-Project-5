#!/usr/bin/env python3
from ev3dev.ev3 import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor import *
from ev3dev2.motor import *
from ev3dev2.button import *
from ev3dev2.sound import *
from array import *
from math import *
import os
from time import *

motorPower = 25 
lowMotorPower = 1
slowMotorPower = 4

homes = [[6 , -6] , [102 , -6] , [6 , 114] , [102 , 114]]
zoneSearchStarts = [[12 , 7.5] , [60 , 7.5] , [12 , 55.5] , [60 , 55.5]]
currentAngle = 0
currentX = 0
currentY = 0
home = -1
targetBox = 1

wheelCirc = 6.88 * pi / 2.54

gy = GyroSensor()


eyes = UltrasonicSensor()
colsen = ColorSensor()

steerPair = MoveSteering(OUTPUT_B, OUTPUT_C)
tankPair = MoveTank(OUTPUT_C, OUTPUT_B)
leftMotor = LargeMotor(OUTPUT_B)
rightMotor = LargeMotor(OUTPUT_C)
medMotor = MediumMotor(OUTPUT_A)

btn = Button() 
spkr = Sound()

def gyroCal():
    angle = 100
    rate = 5
    while(angle != 0):
        rate = gy.rate
        sleep(1)
        angle = gy.angle

def GyroReset():
    global gy
    gy.mode = "GYRO-RATE"
    sleep(1)
    gy.mode = "GYRO-ANG"
    sleep(1)

def iseesomething():    
    steerPair.off(brake = True)
    tankPair.off(brake = True)
    spkr.play_file('/home/robot/vscode-hello-python-master/imWalkinHere.wav', 100)
    while eyes.distance_inches < 6.75 or eyes.distance_inches == 255:
        pass

def iseeabarcode():
    steerPair.off(brake = True)
    tankPair.off(brake = True)
    moveTillNoColor(False, lowMotorPower)
    moveDistance(-.1, lowMotorPower, False)
    vals = moveRead(2.25,slowMotorPower)
    return analyze(vals)

def wait():
    btn.wait_for_bump('enter')

def moveTillNoColor(forward = True, s = motorPower):    
    global currentX, currentY
    direction = 1 if forward else -1
    s *= direction
    integral = 0
    derivative = 0
    lastError = 0
    kd = 20
    ki = 20
    kp = 100
    ks = direction * 50
    ks2 = 0   
    error = 0
    steer = 0
    lastPos = leftMotor.position
    while colsen.reflected_light_intensity > 0:
        error = (gy.angle - currentAngle) * .5 
        objectseen = eyes.distance_inches
        if(objectseen < 6):
            iseesomething()
        
        integral += + error
        derivative = lastError - error
        steer = kp*error + ki*integral + kd*derivative + ks2
        steer /= ks
           
        steerPair.on(steer if (steer < 100 and steer > -100) else (100 if steer > 0 else -100), s)

        pos = leftMotor.position
        dmoved = (pos - lastPos) * wheelCirc / 360
        currentX += dmoved * sin(currentAngle * pi / 180)
        currentY += dmoved * cos(currentAngle * pi / 180) 
        lastPos = pos 
        
        lastError = error        
    steerPair.off(brake = False)
    print("imhere")
    sleep(.5)  

def moveRead(d = 20, s = motorPower):    
    global currentX, currentY
    vals = []
    direction = 1 if d > 0 else -1
    s *= direction
    startAngle = leftMotor.position
    integral = 0
    derivative = 0
    lastError = 0
    kd = 20
    ki = 20
    kp = 100
    ks = direction * 50
    ks2 = 0   
    error = 0
    steer = 0
    dangle =  direction * d * 360 / wheelCirc
    lastPos = startAngle
    while abs(leftMotor.position - startAngle) < dangle:
        vals.append(colsen.reflected_light_intensity)
        error = (gy.angle - currentAngle) * .5 
        objectseen = eyes.distance_inches
        if(objectseen < 6):
            iseesomething()
        
        integral += + error
        derivative = lastError - error
        steer = kp*error + ki*integral + kd*derivative + ks2
        steer /= ks
           
        steerPair.on(steer if (steer < 100 and steer > -100) else (100 if steer > 0 else -100), s)

        pos = leftMotor.position
        dmoved = (pos - lastPos) * wheelCirc / 360
        currentX += dmoved * sin(currentAngle * pi / 180)
        currentY += dmoved * cos(currentAngle * pi / 180) 
        lastPos = pos 

        lastError = error        
    steerPair.off(brake = False)
    sleep(.5)
    return vals

def moveDistance(d = 20, s = motorPower, dontIgnoreColor = True, ignoreEyes = False): 
    spkr.speak("FUCK")   
    global currentX, currentY,currentAngle
    direction = 1 if d > 0 else -1
    s *= direction
    startAngle = leftMotor.position
    integral = 0
    derivative = 0
    lastError = 0
    kd = 20
    ki = 20
    kp = 100
    ks = 50
    ks *= direction
    ks2 = 0   
    error = 0
    steer = 0
    dangle =  direction * d * 360 / wheelCirc
    lastPos = startAngle
    boxPos = -50 * 360 / wheelCirc
    boxtol = 5 * 360 / wheelCirc
    if  abs(gy.angle - currentAngle) > 45:
        currentAngle %= 360
        currentAngle -= 360
    while abs(leftMotor.position - startAngle) < dangle:
        error = (gy.angle - currentAngle) * .5 
        print(error)
        
        objectseen = eyes.distance_inches
        if(objectseen < 6 and not ignoreEyes):
            iseesomething()
        color = colsen.reflected_light_intensity if dontIgnoreColor else -1
        if(color > 0 and (leftMotor.position - boxPos >boxtol)):
            steerPair.off(brake = False)
            boxPos = leftMotor.position
            boxID = iseeabarcode()
            if boxID == targetBox:
                return True
        
        integral += error
        derivative = lastError - error
        steer = kp*error + ki*integral + kd*derivative + ks2
        steer /= ks
           
        steerPair.on(steer if (steer < 100 and steer > -100) else (100 if steer > 0 else -100), s)

        pos = leftMotor.position
        dmoved = (pos - lastPos) * wheelCirc / 360
        currentX += dmoved * sin(currentAngle * pi / 180)
        currentY += dmoved * cos(currentAngle * pi / 180) 
        lastPos = pos       

        lastError = error
    steerPair.off(brake = True)
    sleep(.5)
    adjustDistance = (dangle - (leftMotor.position - startAngle)) * wheelCirc / 360
    if abs(adjustDistance) > 0.01 and dontIgnoreColor:
        moveDistance(adjustDistance, lowMotorPower, False)
    return False
    
    
def movetoxy(x, y):
    print(x,y)
    moveclosest(x,y)    
    dothemove(x,y)

def moveclosest(x, y):
    minx = 6
    miny = 6
    mind = getdistance(x,y,6,6)

    for i in range(5):
        for j in range(3):
            tempx = j * 48 + 6
            tempy = i * 12 + 6
            temp = getdistance(x, y, tempx, tempy)
            if temp < mind:
                minx = tempx
                miny = tempy
                mind = temp
    print(minx, miny)
    if(abs(currentX - minx) <0.1 and abs(currentY - miny) <0.1):
        pass
    else:
        dothemove(minx, miny)

def dothemove(x,y):
    print(currentX, currentY)
    if abs(currentX - x) > .1:
        if x < currentX:
            turnTo(270)
        else:
            turnTo(90)
        dx = abs(x - currentX)
        print(dx, "tacos")
        moveDistance(dx, motorPower if dx > 6.5 else slowMotorPower* 1.5, False, True if dx < 6.5 else False)

    if abs(currentY - y) > .1:
        if(y < currentY):
            turnTo(180)
        else:
            turnTo(0)
        dy = abs(y - currentY)
        moveDistance(dy, motorPower if dy > 6.5 else slowMotorPower * 1.5, False, True if dy < 6.5 else False)

def getdistance(x1,y1,x2,y2):
    dx = x2-x1
    dy = y2-y1
    d2 = dx * dx + dy * dy
    return sqrt(d2)

def turnTo(tangle):
    global currentAngle
    tangle = tangle % 360
    cangle = gy.angle % 360
    delta = tangle - cangle
    if delta > 180:
        delta -= 360
    elif delta < -180:
        delta += 360
    print(delta)
    turnDelta(delta)
    while gy.angle - tangle > 1:
        while gy.angle % 360 <= tangle:
            tankPair.on(lowMotorPower, lowMotorPower * -1)
        while gy.angle % 360 >= tangle:
            tankPair.on(lowMotorPower * -1, lowMotorPower)

    currentAngle = tangle

def turnDelta(delta):
    global currentAngle
    startAng = gy.angle
    if delta > 0:
        tankPair.on(motorPower, motorPower * -1)
        gy.wait_until_angle_changed_by(delta, direction_sensitive = True)
        tankPair.off(brake = True)
        while gy.angle - startAng >= delta:
            tankPair.on(lowMotorPower * -1, lowMotorPower)              
    else:
        tankPair.on(motorPower * -1, motorPower)
        gy.wait_until_angle_changed_by(delta, direction_sensitive = True)
        tankPair.off(brake = True)
        while gy.angle - startAng <= delta:
            tankPair.on(lowMotorPower, lowMotorPower * -1)
        
    tankPair.off(brake = True)
    currentAngle += delta
    currentAngle %= 360

def analyze(data):
    maxval = float(max(data))
    if maxval == 0:
        return 0
    divisions = 5
    subsize = len(data) / divisions
    avgs = []
    relavgs = []
    for i in range(divisions):
        avgs.append(getavg(list(data[int(subsize * i):int(subsize * (i+1))])))
        relavgs.append(avgs[i]/maxval)
    if (relavgs[3] - relavgs[2] > .4):
        print("3")
        return 3
    elif(relavgs[3] > relavgs[2]) and (abs(relavgs[4] - relavgs[3]) < .1):
        print("1")
        return 1
    elif(relavgs[0] < .9 and relavgs[1] < .9 and relavgs[2] < .9 and relavgs[3] < .9 and relavgs[4] < .9):
        print("2")
        return 2
    else:
        print("4")
        return 4
    
def getavg(l):
    sum = 0
    for i in l:
        sum += i
    return sum / len(l)

def navigateZone(start):
    ydelts = [0,21,24,45]
    print(start)
    for i in range(4):
        movetoxy(start[0] + (36 * (i % 2)), start[1] + ydelts[i])
        turnTo(90 + (180 * (i % 2)))
        foundit = moveDistance(36)
        if foundit:
            return True
    return False

def getBox():
    global currentX, currentY
    turnDelta(-90)
    moveDistance(3.5, slowMotorPower, False)
    medMotor.on_for_degrees(motorPower * -1,900)
    moveDistance(-5.9, slowMotorPower, False)
    movetoxy(homeX, homeY)
    turnTo(180)
    moveDistance(6,slowMotorPower,False)
    medMotor.on_for_degrees(motorPower, 900)
    moveDistance(-6, slowMotorPower, False)
    turnDelta(180)
    currentX = homeX
    currentY = homeY

def igps():
    r1 = int(input("Enter distance from A: "))
    r2 = int(input("Enter distance from C: "))
    r3 = int(input("Enter distance from D: "))
    x1 = homes[0][0]
    y1 = homes[0][1]
    x2 = homes[2][0]
    y2 = homes[2][1]
    x3 = homes[3][0]
    y3 = homes[3][1]
    igx = (-(y2 - y3)*((y2^2-y1^2)+(x2^2-x1^2)+(r1-r2)) + (y1-y2)*((y3^2-y2^2)+(x3^2-x2^2)+(r2-r3)))/(2*((x1-x2)*(y2-y3) - (x2-x3)*(y1-y2)))
    igy = (-(x2 - x3)*((x2^2 - x1^2) + (y2^2 - y1^2) + (r1 - r2))+((x1-x2)*((x3^2-x2^2)+(y3^2-y2^2)+(r2-r3))))/(2*((y1-y2)*(x2-x3) - (y2-y3)*(x1-x2)))
    #spkr.play_file('Click')
    print("R1:", r1)
    print("R2:", r2)
    print("R3:", r3)
    print("X:", 35)
    print("Y:", 73)
  
gyroCal()

#main
home = int(input("Enter Home (A/1 B/2 C/3 D/4): "))
home -= 1
currentX = homes[home][0]
currentY = homes[home][1]
homeX = currentX
homeY = currentY
currentAngle = 0
cont = 'y'
boxFound = False
while cont == 'y' or cont == 'Y':
    targetBox = int(input("Enter target box (1/2/3/4): "))
    if targetBox == 0:
        igps()
        continue
    for i in range(4):
        if(currentX - homeX <0.1 and currentY - homeY <0.1):
            moveclosest(homeX, homeY)
        boxFound = navigateZone(zoneSearchStarts[(i + home) % 4])
        if(boxFound):
            getBox()
            print("Box Delivered.")
            cont = input("Retrieve another box (Y/N): ")
            break

def oldCode():
    pass
    # #this calculated the average angle that both motors have rotated
    # def steerPos():
    #     leftRot = leftMotor.position
    #     rightRot = rightMotor.position
    #     return (leftRot + rightRot) / 2

    # #this resents the current angle that both motors have rotated to 0
    # def steerReset():
    #     leftMotor.position = 0
    #     rightMotor.position = 0

    # #this method determines which direction the robot needs to face to move to its new coordinate, then returns that angle
    # # north = 0   (no change in x, increase in y)
    # # east = 90   (increase in x,  no change in y)
    # # south = 180 (no change in x, decrease in y)
    # # west = 270  (decrease in x,  no change in y)
    # #if the robot doesn't move, it returns north
    # def getDirection(oldX, oldY, newX, newY):
    #     if(oldX == newX):
    #         if(oldY < newY):
    #             return 0
    #         elif (oldY > newY): #oldY > newY
    #             return 180
    #         else: #oldX = newX, oldY = newY -> dont move
    #             return 0
    #     else: #oldX != newX, oldY == newY
    #         if(oldX < newX):
    #             return 90
    #         else: #oldX > newX
    #             return 270

    # #this method is used when the robot is in debug mode. 
    # #it will print that it is waiting, and how many times it has waited during this run of the program
    # #the program then waits for the center, or 'enter' button on the robot to be pressed before continueing
    # i = 0
    # def debugWait():
    #     global i
    #     i = i + 1
    #     print("Waiting: ", str(i))    
    #     btn.wait_for_bump('enter')
    #     sleep(.1)


    # def calculateAngleProper(inAngle):
    #     angle = degrees(acos(cos(radians(inAngle))))
    #     if(inAngle % 360) > 180:
    #         angle = angle * -1
    #     return round(angle, 0)


    # #this method will rotate the robot to face (newX, newY) from its current position (oldX, oldY), given that one 
    # #of the coordinates is the same. The robot will only spin clockwise
    # def rotateToDirection(oldX, oldY, newX, newY, currentDirection):
    #     #calculates how far the robot needs to turn
    #     direction = getDirection(oldX, oldY, newX, newY)
    #     delta = (direction - currentDirection) % 360
    #     delta = calculateAngleProper(delta)
    #     spinDirection = 0
    #     if delta == 0:
    #         spinDirection = 1
    #     else:
    #         spinDirection = delta / abs(delta)

    #     #begins turning robot clockwise until it has turned delta degrees
    #     tankPair.on((motorPower * spinDirection), (motorPower * -1 * spinDirection))
    #     gy.wait_until_angle_changed_by(delta, direction_sensitive = True)
    #     tankPair.off(brake=True)
    #     sleep(.25)

    #     #the robot usually overshoots the turn, so this rotates the robot counter-clockwise much slower than clockwise.
    #     #this is done until it reaches the angle it was supposed to turn to originally.
    #     tankPair.on(lowMotorPower * -1 * spinDirection, lowMotorPower * spinDirection)
    #     if abs(delta) == 180:
    #         while (gy.angle - currentDirection) % 360 > abs(delta):
    #             pass
    #     else: 
    #         while abs(calculateAngleProper(gy.angle - currentDirection)) > abs(delta):
    #             pass
    #     tankPair.off(brake = True)
    #     sleep(.25)

    #     #if the program is in debug mode, the robot will wait for the center button to be pressed before continuing
    #     if(debug):
    #         debugWait()

    #     return direction

    # #the method inputs the direction the robot should currently be facing
    # #   0 = N, 90 = E, 180 = S, 270 = W
    # #and measures the current reading on the gyro sensor. then it calculates what the steering value needs
    # #to be for the robot to adjust to any drifting while driving
    # #if the robot veers slightly left of its intended path, this will return a steering value to turn it slightly right
    # def getAngle(direction):
    #     angle = gy.angle
    #     angle = (angle - direction) % 360
    #     if(angle > 180):
    #         return ((abs(angle - 360)) * 100 / 180)
    #     else:
    #         return (angle * 100 / 180) * -1

    # #moves robot from (oldX, oldY) to (newX, newY) given that either the x or y coordinate doesn't change
    # #if both do change, an if block in the main code will ensure this is called twice,
    # #   once to move along X, then once to move along Y
    # def moveCord(oldX, oldY, newX, newY, currentDirection):
    #     #call to rotateToDirection to properly orient the robot before moving forward
    #     direction = rotateToDirection(oldX, oldY, newX, newY, currentDirection)

    #     #calculates the distance the robot needs to travel in terms of degrees the motors need to rotate
    #     distance = abs((newX - oldX) + (newY - oldY))
    #     distanceAngle = distance * 360 / wheelCirc
    #     steerReset()

    #     #moves the robot forward until the motors have rotated the previously calculate number of degrees
    #     #gyro sensor is continually checked to ensure a straight path
    #     while abs(steerPos()) < distanceAngle:
    #         steerPair.on(getAngle(direction), motorPower)
    #     steerPair.off(brake = True)

    #     #due to inertia, the robot tends to overshoot the distance. this block moves the robot either 
    #     #forward (undershoot) or backward (overshoot) at a much slower speed to adjust for this
    #     remainingAngle = 2
    #     while (abs(remainingAngle) > 1):
    #         remainingAngle = distanceAngle - steerPos()
    #         steerPair.on_for_degrees(getAngle(direction), lowMotorPower, remainingAngle)
    #     steerPair.off(brake = True)
    #     sleep(.5)

    #     #if the program is in debug mode, the robot will wait until the center button is pressed before continueing
    #     if(debug):
    #         debugWait()

    #     return direction

    # #main code
    # #a for loop goes through every set of coordinates in the cords array and gets the current (x,y) and next (x,y) of the robot.
    # #because the robot can only move directly along the x or y axis, for every move, either oldX = newX OR oldY = newY.
    # #if that is not the case, and the robot must move to another point that isnt orthagonally related to it, the robot will
    # #move twice, once along the x, then once along the y. if this measure were not taken, it is unclear what the robot would do.
    # #if the next coordinate it is going to is (0,0), then the robot has reached the desired final point and will reorient itself
    # #to face +y, then wait the given amound of time, before moving to (0,0)
    # #after it has gone through all the points, it will reorient itself to face +y
    # def main():
    #     currentDirection = 0
    #     for j in range(len(cords[0]) - 1):
    #         oldX = cords[0][j]
    #         oldY = cords[1][j]
    #         newX = cords[0][j + 1]
    #         newY = cords[1][j + 1]
    #         if(newX == 0 and newY == 0):
    #             currentDirection = rotateToDirection(oldX, oldY, oldX, oldY, currentDirection)
    #             sleep(waitTimeAtEnd)    
    #         if(oldX != newX and oldY != newY):
    #             currentDirection = moveCord(oldX, oldY, oldX, newY, currentDirection)
    #             currentDirection = moveCord(oldX, newY, newX, newY, currentDirection)
    #         else:
    #             currentDirection = moveCord(oldX, oldY, newX, newY, currentDirection)
    #     currentDirection = rotateToDirection(0, 0, 0, 0, currentDirection)        


    # print("Ready to Start")
    # btn.wait_for_bump('enter')
    # main()


#brickrun -r ./vscode-hello-python-master/PID.py
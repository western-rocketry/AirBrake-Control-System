# -*- coding: utf-8 -*-
"""
Created on Tue May 15 19:12:24 2018

@author: rjeff

Air Brake Control System

--Records flight data - Predicts apogee and if it is determined that the rocket will surpass 10,000ft
the program will determine how far to deploy airbrakes 
"""

import time
from math import exp, log, sqrt, pow

from accelerometer import Accelerometer
from altimeter import Altimeter

import numpy as np
import os
import RPi.GPIO as GPIO


#constants that will be used in calculation later

MM = 0.02869 #Molar Mass of Air (Kg/mol) (Average of dry and humid air values)
Rc = 8.3  #Universal ideal gas constant (J/(mol*Kg))

Lr = 0.0065 #(K/m) Temperature Lapse Rate
stPress = 101325 #(Pa) standard sea level pressure
Gc = 9.8 #(m/s^2) Gravitational constant
stTemp = -288.15 #standard temp (negative because that is what is needed according to the equation, this is easier)

mass = 333 #TBD dry mass
Cd = 0.45 #VERIFY LATER -- Coeffecient of drag
Acr = 0.0154283935 #(m^2) Rocket Cross-sectional area

def airDensity_height (pressure, temperature):
    
     press = pressure 
     
     #height calculation will be done in 3 parts, deconstructed from memo     
     pOne = (log((press/stPress)))*((Rc*Lr) / (Gc*MM))
     pTwo = exp(pOne)
     hgt = (stTemp*(pTwo - 1))/Lr #height final value
     
     temp = 288.15 - (Lr*hgt)
  
     dens = (press*MM)/(Rc*temp)
     
     return dens, hgt

#velocity can be taken off the first integration from accelerometer
def apogeePredictor(density, height, velocity):
    
    densCalc = density
    velo = velocity
    hgtCalc = height
    
    #terminal velocity calculation
    Vterm = sqrt(((2*mass*Gc)/(Cd*Acr*densCalc)))
    
    Vterm_sqrd = pow(Vterm,2)
    
    #apogee calculation 
    apg = ((Vterm_sqrd)/(2*Gc))*(log( ((pow(velo,2)+Vterm_sqrd)/Vterm_sqrd) )) + hgtCalc

    return apg

def getIntegral(valONE, valTWO, timeDiff):
    
    val1 = valONE
    val2 = valTWO
    deltaT = timeDiff
    
    integ = abs( ((val1 + val2) / 2) * deltaT )
    
    return integ

def getDerivative (vlOne, vlTwo, tmDiff):
    
    vl1 = vlOne #reading of 2 cycles past
    vl2 = vlTwo #current reading
    delT = tmDiff
    
    drvtv = (vl2 - vl1) / (2*delT)
    
    return drvtv

def servoCommand(pwLength):
    
    pwLen = (pwLength/1000) ##had a +1 here before... no idea why lol
    #print (pwLen)
    pwm.start(0) #start with a 0 duty cycle, doesnt set any angle on start up
    
    duty = (pwLen/20)*100
    #print (duty)

    GPIO.output(11, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.005) 
    GPIO.output(11,False)
    pwm.ChangeDutyCycle(0)
    
    pwm.stop()
        

def AirBrake_Control(time, apogee, iteration, agrInteg, errorSignal):
    
    predError = apogee - 3048 #as per the Rocket Calculations Memo
    
    currSig[0] = time, predError
    
    if (iteration <= 2):
        errorSignal[iteration] = currSig[0]
        PID = 9
        
    else:
        #the next two lines are essentially shifting values as per rocket memo
        errorSignal = np.concatenate((errorSignal, currSig))
#        print (errorSignal)
        errorSignal = np.delete(errorSignal, 0, 0)
        
        iTmDiff = errorSignal[1,0] - errorSignal[0,0]  #get time diff
        intg = getIntegral(errorSignal[1,1], errorSignal[0,1], iTmDiff)  #get integral
        agrInteg = agrInteg + intg  #aggregate integral value
        
        dTmDiff = errorSignal[2,0] - errorSignal[0,0]  #get time diff
        drvt = getDerivative(errorSignal[0,1], errorSignal[2,1], dTmDiff)
        
        #PID control equation
        PID = ( (0.2)*errorSignal[1,1] + (0.2)*agrInteg + (0.2)*drvt ) + 1
    
#    print (errorSignal)
#    print ("________")
    
    return PID, agrInteg
        

#------------program system start------------#
    
#sensor objects
acc = Accelerometer()
alt = Altimeter()

#start clock count
tm = time.time()

#make dir to record flight data
root = "/home/pi/Documents/Flight Data/"
stamp = time.asctime(time.localtime(time.time()))
os.makedirs(root+stamp)

#to store max values for use in determining when to initiate controls
vMax = 0
hMax = 0

#for the storage of the error signals of previous cycles || structure: index | time, error
errSig = np.zeros((3,2))
#dummy array of current signal for concatenation || structure: time, error
currSig = np.zeros((1,2))

#to store the summation of all previous integral results from PID
aggregatedInteg = 0

#set rPi for servo control
GPIO.setmode(GPIO.BOARD) #before we can start naming pins, this sets the pin as per the schem.

GPIO.setwarnings(False)
GPIO.setup(11,GPIO.OUT)

pwm = GPIO.PWM(11,50) #pin 11, 50 Hz


itrCount = 0 #count iteration
runCon = 1
while (runCon==1):
    
    #structure: time, pressure, temerature, velocity, density, height, predicted apogee
    interimResult = np.zeros([1,7])
    
    readCon = 1
    while (readCon==1):
        
        try:
            a0, tm0 = acc.readACC()
            interimResult[0,0], interimResult[0,1], interimResult[0,2] = alt.readALT()
            interimResult[0,0] = interimResult[0,0] - tm
            a1, tm1 = acc.readACC()
            
            readCon = 0
            
        except OSError:
            
            print ("Lost Connection... Waiting")
            time.sleep(0.05)
            
    
    #record values needed to predict apogee (see interimResult structure)
    deltaTime = tm1 - tm0
    interimResult[0,3] = getIntegral(a0,a1,deltaTime) #get velocity
    interimResult[0,4], interimResult[0,5] = airDensity_height(interimResult[0,1], interimResult[0,2])
    
    #predict apogee
    interimResult[0,6] = apogeePredictor(interimResult[0,4], interimResult[0,5], interimResult[0,3])
    
    '''
    #save values 
    np.save((root+stamp+"/"+str(itrCount)+".npy"),interimResult)
    
    '''
    pidSig, aggregatedInteg = AirBrake_Control(interimResult[0,0], interimResult[0,6], itrCount, aggregatedInteg, errSig)
    
    #the following two conditions ensures that airbrakes are only being deployed after engine shutoff
    if (vMax < interimResult[0,3]):
        vMax = interimResult[0,3]
        aggregatedInteg = 0 #prevent integral values from being aggregated before engine shutoff
    
    elif ( (vMax - interimResult[0,3]) > 10 ): ##lower or more than 10?? will this be a problem with the aggregated integral
        
        try:
            ##---------vvv---------This condition is no longer necessary consider removing
            if (pidSig!=9):
                servoCommand(pidSig)
                
        except ValueError:
            print("Negative value")
            
    #program exit condition
    if (hMax < interimResult[0,5]):
        hMax = interimResult[0,5]
    ###########The delay before needs to change ---- Brakes must be in before parachute deployment
    elif (hMax > (interimResult[0,5] + 10)): #10 here is arbitrary but should be fine... After max height, if height is 10m below bring airbrakes in and stop program
        servoCommand(0)
        runCon = 0
        
    itrCount = itrCount+1

GPIO.cleanup()

np.save((root+stamp+"/Maximum Velocity.npy"),vMax)
np.save((root+stamp+"/Maximum Height.npy"),hMax)

        

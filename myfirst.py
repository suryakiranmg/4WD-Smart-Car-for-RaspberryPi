########################################################################
# Filename    : SimpleUltrasonicLED.py
# Description : Get distance via UltrasonicRanging sensor & ON/OFF leds in sequence
# author      : Surya
########################################################################
import RPi.GPIO as GPIO
import time

redLED =11
greenLED = 13
blueLED = 15

trigPin = 16
echoPin = 18
MAX_DISTANCE = 220          # define the maximum measuring distance, unit: cm
timeOut = MAX_DISTANCE*60   # calculate timeout according to the maximum measuring distance

def pulseIn(pin,level,timeOut): # obtain pulse time of a pin under timeOut
    t0 = time.time()
    while(GPIO.input(pin) != level):
        if((time.time() - t0) > timeOut*0.000001):
            return 0;
    t0 = time.time()
    while(GPIO.input(pin) == level):
        if((time.time() - t0) > timeOut*0.000001):
            return 0;
    pulseTime = (time.time() - t0)*1000000
    return pulseTime
    
def getSonar():     # get the measurement results of ultrasonic module,with unit: cm
    GPIO.output(trigPin,GPIO.HIGH)      # make trigPin output 10us HIGH level 
    time.sleep(0.00001)     # 10us
    GPIO.output(trigPin,GPIO.LOW) # make trigPin output LOW level 
    pingTime = pulseIn(echoPin,GPIO.HIGH,timeOut)   # read plus time of echoPin
    distance = pingTime * 340.0 / 2.0 / 10000.0     # calculate distance with sound speed 340m/s 
    return distance
    
def setup():
    GPIO.setmode(GPIO.BOARD)      # use PHYSICAL GPIO Numbering
    GPIO.setup(trigPin, GPIO.OUT)   # set trigPin to OUTPUT mode
    GPIO.setup(echoPin, GPIO.IN)    # set echoPin to INPUT mode
    GPIO.setup(redLED, GPIO.OUT)
    GPIO.setup(greenLED, GPIO.OUT)
    GPIO.setup(blueLED, GPIO.OUT)
    GPIO.output(redLED, GPIO.HIGH)
    GPIO.output(greenLED, GPIO.HIGH)
    GPIO.output(blueLED, GPIO.HIGH)

def loop():
    while(True):
        distance = getSonar() # get distance
        print ("The distance is : %.2f cm"%(distance))
        if(distance <= 6):
            GPIO.output(redLED, GPIO.LOW)
            GPIO.output(greenLED, GPIO.HIGH)
            GPIO.output(blueLED, GPIO.HIGH)        
        elif(distance > 6 and distance <= 12):
            GPIO.output(redLED, GPIO.HIGH)
            GPIO.output(greenLED, GPIO.LOW)
            GPIO.output(blueLED, GPIO.HIGH)
        
        elif(distance >12 and distance <= 18):
            GPIO.output(redLED, GPIO.HIGH)
            GPIO.output(greenLED, GPIO.HIGH)
            GPIO.output(blueLED, GPIO.LOW)
        elif(distance >18):
            GPIO.output(redLED, GPIO.HIGH)
            GPIO.output(greenLED, GPIO.HIGH)
            GPIO.output(blueLED, GPIO.HIGH)
        
        #time.sleep(1)
        
if __name__ == '__main__':     # Program entrance
    print ('Program is starting...')
    setup()
    try:
        loop()
    except KeyboardInterrupt:  # Press ctrl-c to end the program.
        GPIO.cleanup()         # release GPIO resource


    

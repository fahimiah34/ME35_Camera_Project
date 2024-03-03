import numpy as np
import cv2
from picamera2 import Picamera2
from libcamera import controls
import time
import RPi.GPIO as GPIO

picam2 = Picamera2() # assigns camera variable
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous}) # sets auto focus mode
picam2.start() # activates camera

time.sleep(1) # wait to give camera time to start up

# motor 
ena = 3
in1 = 29
in2 = 31
in3 = 33
in4 = 35
enb = 5

# Setup GPIO and pins
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
# motor
GPIO.setup(ena, GPIO.OUT) 
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(enb, GPIO.OUT)

# control speed of motors

SPEED = 18

p1 = GPIO.PWM(ena, 50)
p2 = GPIO.PWM(enb, 50)
p1.ChangeDutyCycle(SPEED)
p2.ChangeDutyCycle(SPEED)
p1.start(0)
p2.start(0)

def RunMotor():
    p1.ChangeDutyCycle(SPEED)
    p2.ChangeDutyCycle(SPEED)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

def TurnMotorRight(duty_cycle):
    p1.ChangeDutyCycle(duty_cycle)#SPEED + duty_cycle)
    p2.ChangeDutyCycle(0)#SPEED - duty_cycle)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

def TurnMotorLeft(duty_cycle):
    p1.ChangeDutyCycle(0)#SPEED - duty_cycle)
    p2.ChangeDutyCycle(duty_cycle)#SPEED + duty_cycle)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

def StopMotor():
    p1.ChangeDutyCycle(0)
    p2.ChangeDutyCycle(0)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

try:
    error_sum = 0
    previous_error = 10
    while True:
        
        # Display camera input
        image = picam2.capture_array("main")
        cv2.imshow('img',image)
    
        # Crop the image
        crop_img = image[0:2500, 0:4600]
    
        # Convert to grayscale
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
    
        # Gaussian blur
        blur = cv2.GaussianBlur(gray,(5,5),0)
    
        # Color thresholding
        input_threshold,comp_threshold = cv2.threshold(blur,60,255,cv2.THRESH_BINARY_INV)
    
        # Find the contours of the frame
        contours,hierarchy = cv2.findContours(comp_threshold.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
        # Find the biggest contour (if detected)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c) # determine moment - weighted average of intensities

            if int(M['m00']) != 0:
                cx = int(M['m10']/M['m00']) # find x component of centroid location
                cy = int(M['m01']/M['m00']) # find y component of centroid location
            else:
                print("Centroid calculation error, looping to acquire new values")
                continue
            cv2.line(crop_img,(cx,0),(cx,720),(255,0,0),1) # display vertical line at x value of centroid
            cv2.line(crop_img,(0,cy),(1280,cy),(255,0,0),1) # display horizontal line at y value of centroid
    
            cv2.drawContours(crop_img, contours, -1, (0,255,0), 2) # display green lines for all contours
            
            # determine location of centroid in x direction and adjust steering recommendation
            high_value = 420
            low_value = 220
            set_point = 315
            current_value = cx
            error = set_point - current_value
            p = error * 0.001
            p = max(min(p, 25), 15)

            #i = error_sum * 0.0005

            d = (error - previous_error) * 0.05

            pid = p #+ d 
            #pid = max(min(pid, 18), 15)
            #if pid < 0: 
            #    pid = 0 
            #if pid > SPEED: 
            #    pid = SPEED
            print(pid)
            if cx >= high_value:
                TurnMotorLeft(pid)
                #time.sleep(0.05)
                print("Turn Left!")
            elif cx <= low_value:
                TurnMotorRight(pid)
                #time.sleep(0.05)
                print("Turn Right")
            else:
                RunMotor()
                print("On Track!")
            print(cx)
    
        else:
            print("I don't see the line")
    
        # Display the resulting frame
        cv2.imshow('frame',crop_img)
        
        # Show image for 1 ms then continue to next image
        cv2.waitKey(1)

        previous_error = error
        error_sum += error 

except KeyboardInterrupt:
    print('All done');
    GPIO.cleanup()

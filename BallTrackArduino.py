# import the necessary packages
from picamera.array import \
    PiRGBArray  # As there is a resolution problem in raspberry pi, will not be able to capture frames by VideoCapture
from picamera import PiCamera
import RPi.GPIO as GPIO
import serial
import time
import cv2
import cv2.cv as cv
import numpy as np


# Image analysis work
def segment_colour(frame):  # returns only the red colors in the frame
    hsv_roi = cv2.cvtColor(frame, cv2.cv.CV_BGR2HSV)
    mask_1 = cv2.inRange(hsv_roi, np.array([160, 160, 10]), np.array([190, 255, 255]))
    ycr_roi = cv2.cvtColor(frame, cv2.cv.CV_BGR2YCrCb)
    mask_2 = cv2.inRange(ycr_roi, np.array((0., 165., 0.)), np.array((255., 255., 255.)))

    mask = mask_1 | mask_2
    kern_dilate = np.ones((8, 8), np.uint8)
    kern_erode = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, kern_erode)  # Eroding
    mask = cv2.dilate(mask, kern_dilate)  # Dilating
    # cv2.imshow('mask',mask)
    return mask


def find_blob(blob):  # returns the red colored circle
    largest_contour = 0
    cont_index = 0
    contours, hierarchy = cv2.findContours(blob, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for idx, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > largest_contour):
            largest_contour = area

            cont_index = idx
            # if res>15 and res<18:
            #    cont_index=idx

    r = (0, 0, 2, 2)
    if len(contours) > 0:
        r = cv2.boundingRect(contours[cont_index])

    return r, largest_contour


def target_hist(frame):
    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    hist = cv2.calcHist([hsv_img], [0], None, [50], [0, 255])
    return hist


# CAMERA CAPTURE
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (160, 120) #faster matrix for raspberry pi 2
camera.framerate = 16
rawCapture = PiRGBArray(camera, size=(160, 120))

# initialize Arduino for controlling the servo and brushless
arduino = serial.Serial('/dev/ttyACM0', 115200)

# initialize Servo
search=3
encendido=0
next_cycle=0
andando=0
arranca=0
freno=0
GPIO.setmode(GPIO.BOARD)  # Ponemos la Raspberry en modo BOARD
GPIO.setup(7, GPIO.OUT)  # Ponemos el pin 7 como salida
GPIO.setup(13, GPIO.OUT)
servo1 = GPIO.PWM(7, 50)  # Ponemos el pin 7 en modo PWM y enviamos 50 pulsos por segundo
servo2 = GPIO.PWM(13, 50)
servo1.start(7)  # Enviamos un pulso del 7% para centrar el servo
servo2.start(7)

# allow the camera to warmup
time.sleep(0.001)

# capture frames from the camera
for image in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp and occupied/unoccupied text
    frame = image.array
    frame = cv2.flip(frame, 1)
    global centre_x
    global centre_y
    centre_x = 0.
    centre_y = 0.
    hsv1 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_red = segment_colour(frame)  # masking red the frame
    loct, area = find_blob(mask_red)
    x, y, w, h = loct

    if (w * h) < 10:
        found = 0

        if andando > 0:
            arduino.write('h') #comentar esto para quitar modo andar continuo
            andando=0
            arranca=0

        if encendido<1:
            servo1.start(7)
            servo2.start(7)
            #arduino.write('c')  # comentar esto para quitar modo andar continuo
            encendido=1
            next_cycle=0

        search=search+0.1
        if(search < 12):
            servo2.ChangeDutyCycle(search)
        else:
            search=3
            servo2.ChangeDutyCycle(search)

    else:
        found = 1
        search=3
        encendido=0
        next_cycle+=1
        servo1.ChangeDutyCycle(7)
        servo2.ChangeDutyCycle(7)
        servo1.stop
        servo2.stop
        #simg2 = cv2.rectangle(frame, (x, y), (x + w, y + h), 255, 2) #if you want to see the tarjet in CV windows decoment this
        centre_x = x + ((w) / 2)
        centre_y = y + ((h) / 2)
        #cv2.circle(frame, (int(centre_x), int(centre_y)), 3, (0, 110, 255), -1)#if you want to see the tarjet in CV windows decoment this
        centre_x -= 80
        centre_y = 6 - -centre_y
        print centre_x, centre_y, h

        if next_cycle > 0:

            if centre_x < -24:
                if (h < 30) and (h > 10):
                    arduino.write('k')
                else:
                    arduino.write('a')

            else:
                if centre_x > 24:
                    if (h < 30) and (h > 10):
                        arduino.write('j')
                    else:
                        arduino.write('b')

                else:
                    arduino.write('g')


            if (h < 25) and (h > 10):
                #arduino.write('c')
                #time.sleep(0.7)
                if arranca < 1:         #comentar esto para quitar modo andar continuo
                    arduino.write('f')  #comentar esto para quitar modo andar continuo
                    andando=1           #comentar esto para quitar modo andar continuo
                    arranca=1
                    freno=0
                    print "run"
            else:
                if h >=25 and h < 85:
                    arduino.write('c')
                    time.sleep(0.7)
                    andando=1
                    arranca=0
                    freno=0
                    print "steps"
                else:
                    if h >=85 or h <=10:
                        arduino.write('d')
                        arranca = 0
                        print "stop"

    initial = 400
    flag = 0

    #cv2.imshow("draw",frame) #run CV video windows
    rawCapture.truncate(0)  # clear the stream in preparation for the next frame

    if (cv2.waitKey(1) & 0xff == ord('q')):
        break

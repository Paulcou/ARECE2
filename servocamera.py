import cv2
import numpy as np
import Adafruit_PCA9685
import time
from multiprocessing import Process
from adafruit_servokit import ServoKit

# Initialise the PCA9685 using desired address and/or bus:
pwm = Adafruit_PCA9685.PCA9685(address = 0x40, busnum = 1)
# Set frequency to 60[Hz]
pwm.set_pwm_freq(60)

def servo():
  # Number of servo
  servo_num = 4

  # Configure min and max servo pulse lengths
  servo_min    = 150 # min. pulse length
  servo_max    = 600 # max. pulse length
  servo_offset = 50

  # Set frequency to 60[Hz]
  pwm.set_pwm_freq(60)

  # Move servo on each channel
  for i in range(servo_num):
    print('Moving servo on channel: ', i)
    pwm.set_pwm(i, 0, servo_min + servo_offset)
    time.sleep(1)
  for i in range(servo_num):
    print('Moving servo on channel: ', i)
    pwm.set_pwm(i, 0, servo_max - servo_offset)
    time.sleep(1)
  # Move servo on all channel
  for i in range(servo_num):
    print('Moving servo on channel: ', i)
    pwm.set_pwm(i, 0, servo_min + servo_offset)
    time.sleep(1)
  for i in range(servo_num):
    print('Moving servo on channel: ', i)
    pwm.set_pwm(i, 0, servo_max - servo_offset)
    time.sleep(1)


# Function whichZone to find the area where the plastic cone is placed relative to the car
# return a letter depending on the result
def whichZone(color, x, y, w, h):
  x2 = x + w
  y2 = y + h
  if(color ==  'y'):
    print('Yellow')
    if((y>300 and y<360) or (y2>300 and y2<360)):
      if(x>480 or x2>480):
        print('D')
        return 'D'
      if(x>320 or x2>320):
        print('C')
        return 'C'
      if(x>160 or x2>160):
        print('B')
        return 'B'
      if(x>0 or x2>0):
        print('A')
        return 'A'
    else:
      print('E')
      return 'E'
  else:
   print('Blue')
   if((y>300 and y<360) or (y2>300 and y2<360)):
      if(x<160 or x2<160):
        print('A')
        return 'A'
      if(x<320 or x2<320):
        print('B')
        return 'B'
      if(x<480 or x2<480):
        print('C')
        return 'C'
      if(x<640 or x2<640):
        print('D')
        return 'D'
   else:
     print('E')
     return 'E'

# Process for servomotors
# p1 = Process(target=servo, args=())

kit = ServoKit(channels=16)

# capturing video through webcam
cap = cv2.VideoCapture("/dev/video0")

width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

print('width :', width)
print('height :', height)
# 640 et 480

check = True

contourBleu = False
contourJaune = False

tour = 0
tour2 = 0

angle = 0

checkmoteur = True

print('Angle depart')
print(kit.servo[4].angle)
time.sleep(1)

kit.servo[4].angle = 90
time.sleep(1)
print(kit.servo[4].angle)

# While loop to run the code without end
while(1):
  _, img = cap.read()

  # converting frame(img == BGR) to HSV(hue-saturation-value)
  hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

  # blue color
  blue_lower = np.array([99,80,90],np.uint8)
  blue_upper = np.array([110,255,255],np.uint8)

  # yellow color
  yellow_lower = np.array([22,60,200],np.uint8)
  yellow_upper = np.array([60,255,255],np.uint8)

  # all color together
  blue = cv2.inRange(hsv, blue_lower, blue_upper)
  yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)

  # Morphological Transform, Dilation
  kernal = np.ones((5, 5), "uint8")

  blue = cv2.dilate(blue, kernal)
  res_blue = cv2.bitwise_and(img, img, mask = blue)

  yellow = cv2.dilate(yellow, kernal)
  res_yellow = cv2.bitwise_and(img, img, mask = yellow)

  # Tracking blue
  contours, hierarchy = cv2.findContours(blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  for pic, contour in enumerate(contours):
    area = cv2.contourArea(contour)
    if(area > 300):
      x, y, w, h = cv2.boundingRect(contour)
      img = cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
      cv2.putText(img, "Plot Bleu", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0))
      tour += 1
      if(tour%25==0):
        print(tour)
        zone = whichZone('b', x, y, w, h)
        if(zone == 'A'):
          print('Zone A Blue')
          print(kit.servo[4].angle)
          kit.servo[4].angle = 65
          print(kit.servo[4].angle)
        if(zone == 'B'):
          print('Zone B Blue')
          print(kit.servo[4].angle)
          kit.servo[4].angle = 65
          print(kit.servo[4].angle)
        if(zone == 'C'):
          print('Zone C Blue')
          print(kit.servo[4].angle)
          kit.servo[4].angle = 78
          print(kit.servo[4].angle)
        if(zone == 'D'):
          print('Zone D Blue')
          print(kit.servo[4].angle)
          kit.servo[4].angle = 90
          print(kit.servo[4].angle)

  # Tracking yellow
  contours, hierarchy = cv2.findContours(yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  for pic, contour in enumerate(contours):
      area = cv2.contourArea(contour)
      if(area > 300):
          x, y, w, h = cv2.boundingRect(contour)
          img = cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
          cv2.putText(img, "Plot Jaune", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (40, 80, 220))
          tour2 += 1
          if(tour2%25==0):
            print(tour2)
            zone = whichZone('y', x, y, w, h)
            if(zone == 'D'):
              print('Zone D Yellow')
              print(kit.servo[4].angle)
              kit.servo[4].angle = 115
              print(kit.servo[4].angle)
            if(zone == 'C'):
              print('Zone C Yellow')
              print(kit.servo[4].angle)
              kit.servo[4].angle = 115
              print(kit.servo[4].angle)
            if(zone == 'B'):
              print('Zone B Yellow')
              print(kit.servo[4].angle)
              kit.servo[4].angle = 102
              print(kit.servo[4].angle)
            if(zone == 'A'):
              print('Zone A Yellow')
              print(kit.servo[4].angle)
              kit.servo[4].angle = 90
              print(kit.servo[4].angle)

  img = cv2.rectangle(img, (0, 300), (640, 300), (0, 0, 255), 2)
  img = cv2.rectangle(img, (0, 360), (640, 360), (0, 0, 255), 2)

  cv2.putText(img, "A", (10, 288), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
  cv2.putText(img, "B", (170, 288), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
  cv2.putText(img, "C", (330, 288), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
  cv2.putText(img, "D", (490, 288), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))

  img = cv2.rectangle(img, (160, 300), (160, 360), (0, 0, 255), 2)
  img = cv2.rectangle(img, (320, 300), (320, 360), (0, 0, 255), 2)
  img = cv2.rectangle(img, (480, 300), (480, 360), (0, 0, 255), 2)

  cv2.imshow("Color Tracking", img)

  if(checkmoteur and (tour>0 or tour2>0)):
    pwm.set_pwm(0, 1200, 2896)
    time.sleep(0.5)
    checkmoteur = False

  if cv2.waitKey(10) & 0xFF == ord('q'):
      pwm.set_pwm(0, 0, 4096)
      cap.release()
      cv2.destroyAllWindows()
      break

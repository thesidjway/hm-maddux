from __future__ import division
# Import the PCA9685 module.
import Adafruit_PCA9685
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np
import cv2
from maddux.robots.compute_kine import compute_kine

pwm = Adafruit_PCA9685.PCA9685()
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
servo_min = 200  # Min pulse length out of 4096
servo_max = 650  # Max pulse length out of 4096
pwm.set_pwm_freq(60)
time.sleep(0.1)

def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

def servo_move(channel, angle):
    servo_input = 2.5*angle + 200 # eq of line passing through 0,servo_min and 180,servo_max
    pwm.set_pwm(channel, 0, int(servo_input))
    time.sleep(1)

def servo_to_maddux(theta, link_no):
    if link_no == 1:
	a = 10*(theta - np.pi/3)/9
        print "angle ", a
	return a
    elif link_no == 2:
	a = (5*np.pi - 16*theta)/15
	print "angle, ", a
	return a

def maddux_to_servo(theta, link_no):
    if link_no == 1:
        return 0.9 * theta + np.pi/3
    elif link_no == 2:
        return (-15/16)*(theta - np.pi/3)

servo_move(0, 90)
servo_move(1,150)
servo_move(2,150)
servo_move(3,75)

for raw_frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    goal_in_camera = np.eye(4)
    frame = raw_frame.array
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    res = cv2.aruco.detectMarkers(gray,dictionary)
    # print(res[0],res[1],len(res[2]))

    if len(res[0]) > 0:
        cv2.aruco.drawDetectedMarkers(gray,res[0],res[1])
        print "ID: ", res[1]
        # print(res[0],res[1],len(res[2]))
        # print(res[0])
        cam = np.matrix([[400, 0, 320], [0, 400, 240], [0, 0, 1]])
        rvec, tvec, _  = cv2.aruco.estimatePoseSingleMarkers(res[0], 1.4, cam, np.array([0, 0, 0, 0]))
        rotationvec = cv2.Rodrigues(rvec)[0]
        print(rotationvec)
        print(tvec)
        goal_in_camera[0:3, 0:3] = rotationvec
        goal_in_camera[0:3, 3:4] = tvec.reshape(3,1)
        # theta_1 = theta_2 = theta_3 = theta4 = 0
        goal_config = compute_kine( np.pi/2, servo_to_maddux(2.618, 1), servo_to_maddux(2.618, 2), goal_in_camera)
        print (180/np.pi)*goal_config.ravel().tolist()[0]
	print maddux_to_servo((180/np.pi)*goal_config.ravel().tolist()[1], 1)
	print maddux_to_servo((180/np.pi)*goal_config.ravel().tolist()[2], 2)
	servo_move(0, (180/np.pi)*goal_config.ravel().tolist()[0])
        servo_move(1, maddux_to_servo((180/np.pi)*goal_config.ravel().tolist()[1], 1))
        servo_move(2, maddux_to_servo((180/np.pi)*goal_config.ravel().tolist()[2], 2))
        print "\n\n Goal Config", goal_config
    # Display the resulting frame
    # cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    rawCapture.truncate()
    rawCapture.seek(0)

# When everything done, release the capture
cv2.destroyAllWindows()

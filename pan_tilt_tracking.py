# USAGE
# sudo  python  pan_tilt_tracking.py --cascade haarcascade_frontalface_default.xml

# import necessary packages
from multiprocessing import Manager
from multiprocessing import Process
from imutils.video import VideoStream
from pyimagesearch.objcenter import ObjCenter
from pyimagesearch.pid import PID
import Adafruit_PCA9685 as ada
from tts import speak
#import pantilthat as pth
from elapsed import elapse #add time delay
import argparse
import signal
import time
import sys
import cv2
from convert_deg import cnv #angle value to pwm

pwm= ada.PCA9685(address=0x40, busnum=4)
# define the range for the motors
panServo = (50 , 280)
tltServo = (135 , 200)
pwm.set_pwm_freq(25)
pwm.set_pwm(3, 0, 150) #pan
pwm.set_pwm(13, 0, 175) #tilt

#pwm.set_pwm(1, 0,100)
# function to handle keyboard interrupt
def signal_handler(sig, frame):
	# print a status message
	print("[INFO] You pressed `ctrl + c`! Exiting...")
	# disable the servos pth.servo_enable(1, False)
	#pth.servo_enable(2, False)

	# exit

	sys.exit()

def obj_center(args, objX, objY, centerX, centerY):
	# signal trap to handle keyboard interrupt
        signal.signal(signal.SIGINT, signal_handler)

	# start the video stream and wait for the camera to warm up
        vs = VideoStream(usePiCamera=True).start()
        time.sleep(2.0)
        #cap.set(3 , 640)
	#cap.set(4 , 480)
	# initialize the object center finder
        obj = ObjCenter(args["cascade"])

	# loop indefinitely
        while True:
		# grab the frame from the threaded video stream and flip it
		# vertically (since our camera was upside down)
                frame = vs.read()
		#frame = cv2.flip(frame, 0)

		# calculate the center of the frame as this is where we will
		# try to keep the object
                (H, W) = frame.shape[:2]
                centerX.value = W // 2
                centerY.value = H // 2

		# find the object's location
                objectLoc = obj.update(frame, (centerX.value, centerY.value))
                ((objX.value, objY.value), rect) = objectLoc
                t = time.clock()
		# extract the bounding box and draw it
                if rect is not None:
                        (x, y, w, h) = rect
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0),5)
                        if (w < 150):
                              print ("FAR  .... w = %d " % w)
                             # speak("Please give your valuable feedback")
                        elif (h > 150):
                              print ("NEAR   .... h = %d " % h)
                              #speak("Dear customer, Thanks for visiting our store")
                        
		#else:
                 #    print("rect is none, going to origin")
                  #   elapse(10)
		   #  pwm.set_pwm(3, 0, 280)
                #    pwm.set_pwm(3, 0, 150) #pan
                #    pwm.set_pwm(13, 0, 175) #tilt

                # display the frame to the screen
                cv2.imshow("Pan-Tilt Face Tracking", frame)
		cv2.waitKey(1)

def pid_process(output, p, i, d, objCoord, centerCoord):
	# signal trap to handle keyboard interrupt
	signal.signal(signal.SIGINT, signal_handler)

	# create a PID and initialize it
	p = PID(p.value, i.value, d.value)
	p.initialize()

	# loop indefinitely
        while True:
		# calculate the error
                 error = centerCoord.value - objCoord.value
                 output.value = p.update(error)
                # print('update=', output.value)
def in_range(val, start, end):
	# determine the input vale is in the supplied range
	return (val >= start and val <= end)

def set_servos(pan, tlt):
	# signal trap to handle keyboard interrupt
	signal.signal(signal.SIGINT, signal_handler)

	# loop indefinitely
	while True:
		# the pan and tilt angles are reversed
         	panAngle =  int(pan.value)
                panAngle = cnv(panAngle) + 70
               # print('panAngle=', panAngle)
		tltAngle = int(tlt.value)
		tltAngle= cnv(tltAngle)
               # print('tltAngle=', tltAngle)

		# if the pan angle is within the range, pan
		if in_range(panAngle, panServo[0], panServo[1]):
			 #pth.pan(panAngle)
			#panAngle = cnv(panAngle)
                        pwm.set_pwm(3, 0, panAngle)

		# if the tilt angle is within the range, tilt
		if in_range(tltAngle, tltServo[0], tltServo[1]):
			#tltAngle= cnv(tltAngle)
                        pwm.set_pwm(13, 0, tltAngle)

# check to see if this is the main body of execution
if __name__ == "__main__":
	# construct the argument parser and parse the arguments
	ap = argparse.ArgumentParser()
	ap.add_argument("-c", "--cascade", type=str, required=True,help="path to input Haar cascade for face detection")
	args = vars(ap.parse_args())

	# start a manager for managing process-safe variables
	with Manager() as manager:
		# enable the servos
		#pth.servo_enable(1, True)
		#pth.servo_enable(2, True)

		# set integer values for the object center (x, y)-coordinates
		centerX = manager.Value("i", 0)
		centerY = manager.Value("i", 0)

		# set integer values for the object's (x, y)-coordinates
		objX = manager.Value("i", 0)
		objY = manager.Value("i", 0)

		# pan and tilt values will be managed by independed PIDs
		pan = manager.Value("i", 0)
                print(pan)
		tlt = manager.Value("i", 0)
                print(tlt)
		# set PID values for panning
		panP = manager.Value("f", 0.09)
		panI = manager.Value("f", 0.08)
		panD = manager.Value("f", 0.002)

		# set PID values for tilting
		tiltP = manager.Value("f", 0.11)
		tiltI = manager.Value("f", 0.10)
		tiltD = manager.Value("f", 0.002)

		# we have 4 independent processes
		# 1. objectCenter  - finds/localizes the object
		# 2. panning       - PID control loop determines panning angle
		# 3. tilting       - PID control loop determines tilting angle
		# 4. setServos     - drives the servos to proper angles based
		#                    on PID feedback to keep object in center
		processObjectCenter = Process(target=obj_center,
			args=(args, objX, objY, centerX, centerY))
		processPanning = Process(target=pid_process,
			args=(pan, panP, panI, panD, objX, centerX))
		processTilting = Process(target=pid_process,
			args=(tlt, tiltP, tiltI, tiltD, objY, centerY))
		processSetServos = Process(target=set_servos, args=(pan, tlt))

		# start all 4 processes
		processObjectCenter.start()
		processPanning.start()
		processTilting.start()
		processSetServos.start()

		# join all 4 processes
		processObjectCenter.join()
		processPanning.join()
		processTilting.join()
		processSetServos.join()

		# disable the servos
		#pth.servo_enable(1, False)
		#pth.servo_enable(2, False)

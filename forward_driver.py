#!/usr/bin/env python

import rospy
from ras_lab1_msgs.msg import *
import math

#parameters
v_desired = 0.5 #desired speed
d_desired = 0.45#desired distance from wall

#constants
b = 0.115
r = 0.0352
l = 0.2
pi = 3.141592653589
freq = 10

#variables
de1 = 0 #delta encoder 1
de2 = 0
d1 = 0 #distance for front sensor
d2 = 0
d = 0 #distance between robot and wall
theta = pi/2
vw1 = 0
vw2 = 0
v = 0
w = 0
errsum1 = 0
errsum2 = 0
lasterr1 = 0
lasterr2 = 0
derrsum = 0
lastderr = 0
startbool = 0


def loop():
	listener() #update input variables

	rate = rospy.Rate(freq)
	while not rospy.is_shutdown():
		pw = main()
		talker(pw)
		#print((vw1, vw2, v, w))
		
		rate.sleep()
	rospy.spin()

def main():
	global vw1, vw2, v, w, startbool
	vw1 = 2*pi*r*freq*de1/360 #speed of wheel 1
	vw2 = 2*pi*r*freq*de2/360 #speed of wheel 2
	v = (vw1+vw2)/2 #speed of robot
	w = (vw2-vw1)/(2*b) #angular speed of robot
	
	#v_desired = 0.5
	#w_desired = 0
	#(pw1, pw2) = cart_controller(v_desired, w_desired, vw1, vw2, v, w)
	
	if startbool == 0: #spin first to align then start following wall
		(pw1, pw2) = spinstar()
	else:	
		(pw1, pw2) = wallfollow(d_desired, v_desired)	

	pwm_msg = PWM()
        pwm_msg.PWM1 = pw1
	pwm_msg.PWM2 = pw2

	return pwm_msg #send pwm data to robot

def spinstar():
	global startbool
	thcalc = (theta - pi/2) * 180/pi

	err = (thcalc - 90)/180
	kp = 0.5
	wd = kp*err
	print(theta,abs(thcalc),err*180, wd)
	
	if abs(thcalc) > 80:
		startbool = 1
	(pw1, pw2) = cart_controller(0, wd)

	return(pw1, pw2)

def wallfollow(d_desired, v_desired):
	global v, w, theta, d, derrsum, lastderr
	derr = d - d_desired
	derrsum += derr

	Kp = 0.23
	Ki = 0.0001
	Kd = 0.00014
	
	
	if theta != pi:
		wd = Kp*derr + Ki*derrsum + Kp*(derr - lastderr)
	else:
		wd = 0
	lastderr = derr

	(pw1, pw2) = cart_controller(v_desired, wd)
	return (pw1, pw2)

def cart_controller(vd, wd):
	global errsum1, errsum2, lasterr1, lasterr2, vw1, vw2, v, w
	err1 = (vd-b*wd)-vw1
	err2 = (vd+b*wd)-vw2
	errsum1 += err1
	errsum2 += err2
	dErr1 = err1 - lasterr1
	dErr2 = err2 - lasterr2

	Kp = 250
	Ki = 150
	Kd = 20
	pw1 = Kp*err1 + Ki*errsum1 + Kd*dErr1
	pw2 = Kp*err2 + Ki*errsum2 + Kd*dErr2
	#print(vw1, vw2, v, vd)
	#print(err1, err2, pw1, pw2)
	lasterr1 = err1
	lasterr2 = err2
	
	return (pw1, pw2)

def distance(data):
	global d1, d2, d, theta
	
	if (data.ch1 == 0) and (data.ch2 == 0):
		d1 = 0
		d2 = 0
		d = 0
		theta = pi
	else:
		d1 = 1.114*math.exp(-0.004*data.ch1) #distance for sensor at the back
		d2 = 1.114*math.exp(-0.004*data.ch2) #distance for sensor at the front
		if (d2-d1) != 0:
			gamma = math.atan(l/(d2-d1))
		else:
			gamma = pi/2
		d = d1*math.sin(abs(gamma))
		theta = pi/2 - gamma
	

def encoderdata(data):
	global de1, de2
	de1 = data.delta_encoder1
	de2 = data.delta_encoder2

def listener():

    rospy.init_node('forward_driver', anonymous=True)
    rospy.Subscriber('/kobuki/adc', ADConverter, distance)
    rospy.Subscriber('/kobuki/encoders', Encoders, encoderdata)


def talker(PWMmsg):
    pub = rospy.Publisher('/kobuki/pwm', PWM, queue_size=10) #control PWM of motors
    pub.publish(PWMmsg)


if __name__ == '__main__':
    try:
	loop()
    except rospy.ROSInterruptException:
        pass

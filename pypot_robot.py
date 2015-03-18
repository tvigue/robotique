import itertools
import time
import numpy
import pypot.dynamixel
import json
import math
import kinematics
from contextlib import closing
from pypot.robot import from_json

pos_patte1=[110,0,-60]
pos_patte2=[100,-100,-60]
pos_patte3=[100,100,-60]
pos_patte4=[110,0,-60]
pos_patte5=[100,-100,-60]
pos_patte6=[100,100,-60]

def changeId (dxl_io) :
	# we can scan the motors
	found_ids = dxl_io.scan()  # this may take several seconds
	print 'Detected:', found_ids
	newIds = []
	dxl_io.switch_led_off(found_ids)
	for currentId in found_ids :
		dxl_io.switch_led_on([currentId])
		print 'current Id:', currentId
		
		newId = raw_input("Entrez un nouvelle Id : ")
		
		if newId == '' :
			newId = currentId
		
		newId = int(newId)
		print 'newId:', newId
		
		if newId != currentId :
			dxl_io.change_id({currentId : newId})

		newIds.append(newId)
		print 'newIds:', newIds
		dxl_io.switch_led_off([newId])

	newIds.sort()
	return newIds
	
def angletomotor(theta1, theta2, theta3, found_ids, dxl_io):
	pos = {found_ids[0] : theta1, found_ids[1] : theta2, found_ids[2] : theta3}
	dxl_io.set_goal_position(pos)
	time.sleep(1)  # we wait for 1s
	print 'New pos:', dxl_io.get_present_position(found_ids)
	
def leg_ik (x, y, z, l1=51, l2=63.7, l3=93, alpha=20.69, beta=5.06) :
	rad_alpha  = numpy.deg2rad(alpha)
	rad_beta   = numpy.deg2rad(beta)
	
	d13 = math.sqrt(x**2+y**2)-l1
	d = math.sqrt(d13**2+z**2)
	
	angles = []
	theta1 = math.atan2(y,x)
	theta2 = math.atan2(z,d13) + math.acos((l2**2+d**2-l3**2)/(2*l2*d))
	theta3 = math.pi - math.acos((l2**2+l3**2-d**2)/(2*l2*l3))
	
	angles.append(theta1)
	angles.append(-(theta2 + rad_alpha))
	angles.append(-(theta3 - math.pi/2 + rad_beta + rad_alpha))
	
	return map(lambda x: numpy.rad2deg(x), angles)
	
def setPositionToLeg(x,y,z,leg):
	angles=kinematics.computeIK(x,y,z)
	i=0
	for m in leg:
		m.goal_position = angles[i]
		i=i+1

def allMotorNotCompliant(robot):
	for m in robot.motors:
		m.compliant = False
		
def allMotorCompliant(robot):
	for m in robot.motors:
		m.compliant = True

def allMotorInitialPos(robot):
	print "\n",pos_patte1[0],"\n"
	setPositionToLeg(pos_patte1[0], pos_patte1[1], pos_patte1[2], robot.patte_1)
	setPositionToLeg(pos_patte2[0], pos_patte2[1], pos_patte2[2], robot.patte_2)
	setPositionToLeg(pos_patte3[0], pos_patte3[1], pos_patte3[2], robot.patte_3)
	setPositionToLeg(pos_patte4[0], pos_patte4[1], pos_patte4[2], robot.patte_4)
	setPositionToLeg(pos_patte5[0], pos_patte5[1], pos_patte5[2], robot.patte_5)
	setPositionToLeg(pos_patte6[0], pos_patte6[1], pos_patte6[2], robot.patte_6)
	
def moveRobot(x,y,z,robot):
	print "\n",pos_patte1[0],"\n"
	pos_patte1_tmp=[pos_patte1[0]+y,pos_patte1[1]-x,pos_patte1[2]+z]
	pos_patte2_tmp=[pos_patte2[0]-x,pos_patte2[1]-y,pos_patte2[2]+z]
	pos_patte3_tmp=[pos_patte3[0]-x,pos_patte3[1]-y,pos_patte3[2]+z]
	pos_patte4_tmp=[pos_patte4[0]-y,pos_patte4[1]+x,pos_patte4[2]+z]
	pos_patte5_tmp=[pos_patte5[0]+x,pos_patte5[1]+y,pos_patte5[2]+z]
	pos_patte6_tmp=[pos_patte6[0]+x,pos_patte6[1]+y,pos_patte6[2]+z]
	
	pos_patte1=pos_patte1_tmp
	pos_patte2=pos_patte2_tmp
	pos_patte3=pos_patte3_tmp
	pos_patte4=pos_patte4_tmp
	pos_patte5=pos_patte5_tmp
	pos_patte6=pos_patte6_tmp
	
	setPositionToLeg(pos_patte1[0], pos_patte1[1], pos_patte1[2], robot.patte_1)
	setPositionToLeg(pos_patte2[0], pos_patte2[1], pos_patte2[2], robot.patte_2)
	setPositionToLeg(pos_patte3[0], pos_patte3[1], pos_patte3[2], robot.patte_3)
	setPositionToLeg(pos_patte4[0], pos_patte4[1], pos_patte4[2], robot.patte_4)
	setPositionToLeg(pos_patte5[0], pos_patte5[1], pos_patte5[2], robot.patte_5)
	setPositionToLeg(pos_patte6[0], pos_patte6[1], pos_patte6[2], robot.patte_6)

if __name__ == '__main__':
	
	with closing(pypot.robot.from_json('my_robot2.json')) as robot:
		
		allMotorNotCompliant(robot)
		allMotorInitialPos(robot)
		time.sleep(1)
		while True:
			moveRobot(20,0,0,robot)
			#~ setPositionToLeg(pos_patte1[0], pos_patte1[1]-20, pos_patte1[2], robot.patte_1)
			#~ setPositionToLeg(pos_patte2[0]-20, pos_patte2[1], pos_patte2[2], robot.patte_2)
			#~ setPositionToLeg(pos_patte3[0]-20, pos_patte3[1], pos_patte3[2], robot.patte_3)
			#~ setPositionToLeg(pos_patte4[0], pos_patte4[1]+20, pos_patte4[2], robot.patte_4)
			#~ setPositionToLeg(pos_patte5[0]+20, pos_patte5[1], pos_patte5[2], robot.patte_5)
			#~ setPositionToLeg(pos_patte6[0]+20, pos_patte6[1], pos_patte6[2], robot.patte_6)
			time.sleep(1)
			moveRobot(-20,0,0,robot)
			#~ setPositionToLeg(pos_patte1[0], pos_patte1[1]+20, pos_patte1[2], robot.patte_1)
			#~ setPositionToLeg(pos_patte2[0]+20, pos_patte2[1], pos_patte2[2], robot.patte_2)
			#~ setPositionToLeg(pos_patte3[0]+20, pos_patte3[1], pos_patte3[2], robot.patte_3)
			#~ setPositionToLeg(pos_patte4[0], pos_patte4[1]-20, pos_patte4[2], robot.patte_4)
			#~ setPositionToLeg(pos_patte5[0]-20, pos_patte5[1], pos_patte5[2], robot.patte_5)
			#~ setPositionToLeg(pos_patte6[0]-20, pos_patte6[1], pos_patte6[2], robot.patte_6)
			time.sleep(1)
		
		#~ moveOn(robot)
		#~ setPositionToLeg(150,0,-30,robot.patte_3)
		#~ for m in robot.motors:	
			#~ print 'Current pos:', m.id, ' ', m.present_position
		
		
		#~ allMotorCompliant(robot)
		pass
		


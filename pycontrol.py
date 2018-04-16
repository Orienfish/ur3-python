#######################################################################
# This file is the python API to control UR3 robotic arm.
# Modified by xfyu on Apr 6
#######################################################################
# -*- coding: utf-8 -*-
# !/usr/bin/python
from ctypes import * 
import time

REG_NUM = 6
FORCE = 20
SPEED = 40

# import library
# gcc -o lib.so -fPIC -shared modbustcp.c modbusrtu.c
lib = cdll.LoadLibrary("/home/robot/RL/lib.so")

#
# read_pos - Read 6 pos value
# p[x, y, z, rx, ry, rz]. The x, y, z are in 0.1mm base.
# The rx, ry, rz are in 0.001rad base. 
#
# Parameter: None
# Return value: success - 6 values in m base and rad base
#               fail - None
def read_pos():
	# set the type of parameters
    lib.read_pos.argtype = [POINTER(c_float),] 
    lib.read_pos.restype = c_int
    ret_value = (c_float * REG_NUM)()

  	# read pos position
    res = lib.read_pos(ret_value)
    if not res: # read successfully
    	return ret_value
    return None # fail

#
# read_wrist - Read 6 joint value
# [base, shoulder, elbow, wrist1, wrist2, wrist3].
# All values are in 0.001rad base. 
#
# Parameter: None
# Return value: success - 6 values in rad base
#               fail - None
#
def read_wrist():
	# set the type of parameters
    lib.read_wrist.argtype = [POINTER(c_float),] 
    lib.read_wrist.restype = c_int
    ret_value = (c_float * REG_NUM)()

  	# read pos position
    res = lib.read_wrist(ret_value)
    if not res: # read successfully
    	return ret_value
    return None # fail

#
# send_movel_instruct - send move intruction to robot
#						Using position.
#
# Input: desired_pose - the target position
# Return: None.
#         Error message will display in the Msg box.
#
def send_movel_instruct(desired_pose_pointer):
	# set the type of parameters
    lib.send_movel_instruct.argtype = [POINTER(c_float),] 
    lib.send_movel_instruct(desired_pose_pointer)

#
# send_movej_instruct - send move intruction to robot
#						Using joint positions.
# move to the position decribed by angles' values
#
# Input: float * desired_joint - target position
# Return: none.
#         Error message will display in the Msg box.
#
def send_movej_instruct(desired_joint_pointer):
	# set the type of parameters
    lib.send_movej_instruct.argtype = [POINTER(c_float),] 
    lib.send_movej_instruct(desired_joint_pointer)

#
# gripper_activate - activate the gripper
#
# Speed and force are set in the above
# Return Value: 0 - success
#               -1 - fail
#
def gripper_activate():
	lib.gripper_activate.restype = c_int
	res = lib.gripper_activate()
	return res

#
# gripper_close - close the gripper
#
# Speed and force are set in the above
# Return Value: 0 - success
#               -1 - fail
#
def gripper_close():
	lib.gripper_close.argtype = [c_ubyte, c_ubyte]
	lib.gripper_close.restype = c_int
	res = lib.gripper_close(SPEED, FORCE)
	return res

#
# gripper_open - open the gripper
#
# Speed and force are set in the above
# Return Value: 0 - success
#               -1 - fail
#
def gripper_open():
	lib.gripper_close.argtype = [c_ubyte, c_ubyte]
	lib.gripper_close.restype = c_int
	res = lib.gripper_open(SPEED, FORCE)
	return res	

# main test
if __name__ == '__main__':
	gripper_activate()
	
	'''
	Test 1: read current position and send movl instruction
	'''
	ret = read_pos()
	print "%f %f %f %f %f %f" %(ret[0], ret[1], ret[2], ret[3], ret[4], ret[5])
	ret[2] = ret[2] + 0.05
	send_movel_instruct(byref(ret))
	print "finish test 1"
	time.sleep(3)

	'''
	Test 2: read current wrist angle and send movej instruction
	'''
	ret = read_wrist()
	print "%f %f %f %f %f %f" %(ret[0], ret[1], ret[2], ret[3], ret[4], ret[5])
	ret[2] = ret[2] + 0.3
	send_movej_instruct(byref(ret))
	print "finish test 2"
	time.sleep(3)

	'''
	Test 3: when reach minimum and maximum bound
	'''
	# target angle value
	goal = (c_float*REG_NUM)(5.523,-1.145,0.812,0.815,0.455,0.054)
	gripper_close()
	send_movej_instruct(byref(goal))
	send_movej_screw(DOWN)
	send_movej_screw(UP)
	send_movej_screw(UP)

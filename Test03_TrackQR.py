#!/usr/bin/python

# ===================================================================
# Take off, then go front for 20 seconds or until QR is detected
# When the QR VTOL3 is detected, correct the position and wait until
#  10 second then land
# Control : proportional
# Revision history:
# - going forward for searching QR instead of Hovering
# - create constants for adjustable parameters
# - using rangefinder for altitude instead of EKF altitude
# ===================================================================

import dronekit
import dronekit_sitl
import time
from pymavlink import mavutil
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

# from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
# from pymavlink import mavutil # Needed for command message definitions
# import time
# import math
from rospy import Rate

connection_string = 'udp:10.42.0.66:14551'

SPEED_UD_MAX = 90
SPEED_LR_MAX = 80
SPEED_BF_MAX = 80

TAKE_OFF_THRUST = 1650
TARGET_ALTITUDE = 1.0
MAX_TARGET_ALTITUDE = 3.0

# Tolerance in M
UD_TOLERANCE = 0.05
LR_TOLERANCE = 0.05
BF_TOLERANCE = 0.05

# How far the drone from QR
BF_TARGET = 0.8

use_px = 1
show_QR = 0
# Parameter proportional Kp
KpUD = 250
KpLR = 300
KpBF = 300

# Go front time for searching QR in sec
FRONT_SEARCH_T = 5

# Max min height
MAX_HEIGHT = 5.0
MIN_HEIGHT = 0.5

# RC radio channel
THRUST_CHANNEL = 3
LR_CHANNEL = 1
BF_CHANNEL = 2

# Timing when drone on the target
STABLE_TIME = 15



load1dropped = 0
load2dropped = 0
load3dropped = 0

LRvar = 1500
UDvar = 1500

#######################################################################
# Function s
#######################################################################

# def drop_target(no_target):
# 	if no_target == 0:
# 		msg = vehicle.message_factory.command_long_encode(
# 			0, 0,  # target_system, target_component
# 			mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
# 			0,  # confirmation
# 			12,  # servo number
# 			900,  # servo position between 1000 and 2000
# 			0, 0, 0, 0, 0)  # param 3 ~ 7 not used
# 		load1dropped = 1
#
# 	if no_target==1:
# 		msg = vehicle.message_factory.command_long_encode(
#         0, 0,  # target_system, target_component
#         mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
#         0,  # confirmation
#         12,  # servo number
#         500,  # servo position between 1000 and 2000
#         0, 0, 0, 0, 0)  # param 3 ~ 7 not used
# 		load1dropped = 1
#
# 	if no_target == 2:
# 		msg = vehicle.message_factory.command_long_encode(
# 			0, 0,  # target_system, target_component
# 			mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
# 			0,  # confirmation
# 			12,  # servo number
# 			1500,  # servo position between 1000 and 2000
# 			0, 0, 0, 0, 0)  # param 3 ~ 7 not used
# 		load2dropped = 1
#
# 	if no_target == 3:
# 		msg = vehicle.message_factory.command_long_encode(
# 			0, 0,  # target_system, target_component
# 			mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
# 			0,  # confirmation
# 			12,  # servo number
# 			2300,  # servo position between 1000 and 2000
# 			0, 0, 0, 0, 0)  # param 3 ~ 7 not used
# 		load3dropped = 1
#
# 	# send command to vehicle
# 	vehicle.send_mavlink(msg)


def hover(time_sec):
	counter = time_sec / 0.25
	i = 0
	while i < counter:
		vehicle.channels.overrides[3] = 1500
		vehicle.channels.overrides[1] = 1500
		vehicle.channels.overrides[2] = 1500
		time.sleep(0.25)
		i = i + 1


def hover_once():
	vehicle.channels.overrides[3] = 1500
	vehicle.channels.overrides[1] = 1500
	vehicle.channels.overrides[2] = 1500


def updown_once(vSpeed):
	vehicle.channels.overrides[3] = 1500 + vSpeed
	vehicle.channels.overrides[1] = 1500
	vehicle.channels.overrides[2] = 1500


# minus left, plus right
def left_right_timed(speed, ttime_sec):
	counter = ttime_sec / 0.25
	i = 0
	if speed > SPEED_LR_MAX:
		speed = SPEED_LR_MAX
	elif speed < -SPEED_LR_MAX:
		speed = -SPEED_LR_MAX

	while i < counter:
		vehicle.channels.overrides[3] = 1500
		vehicle.channels.overrides[1] = 1500 + speed
		vehicle.channels.overrides[2] = 1500
		time.sleep(0.25)
		i = i + 1
		if not vehicle.armed:
			vehicle.channels.overrides[3] = 1500
			vehicle.channels.overrides[1] = 1500
			vehicle.channels.overrides[2] = 1500
			break


# minus left, plus right
def left_right_once(speed):
	if speed > SPEED_LR_MAX:
		speed = SPEED_LR_MAX
	elif speed < -SPEED_LR_MAX:
		speed = -SPEED_LR_MAX

	vehicle.channels.overrides[3] = 1500
	vehicle.channels.overrides[1] = 1500 + speed
	vehicle.channels.overrides[2] = 1500
	time.sleep(0.25)


# minus backward, plus forward
def back_forward_timed(speed, ttime_sec):
	counter = ttime_sec / 0.25
	i = 0
	if speed > SPEED_BF_MAX:
		speed = SPEED_BF_MAX
	elif speed < -SPEED_BF_MAX:
		speed = -SPEED_BF_MAX

	while i < counter:
		vehicle.channels.overrides[3] = 1500
		vehicle.channels.overrides[1] = 1500
		vehicle.channels.overrides[2] = 1500 + speed
		time.sleep(0.25)
		i = i + 1
		if not vehicle.armed:
			vehicle.channels.overrides[3] = 1500
			vehicle.channels.overrides[1] = 1500
			vehicle.channels.overrides[2] = 1500
			break


# minus backward, plus forward
def back_forward_once(speed):
	if speed > SPEED_BF_MAX:
		speed = SPEED_BF_MAX
	elif speed < -SPEED_BF_MAX:
		speed = -SPEED_BF_MAX

	vehicle.channels.overrides[3] = 1500
	vehicle.channels.overrides[1] = 1500
	vehicle.channels.overrides[2] = 1500 + speed
	time.sleep(0.25)



def correct_altitude(x, y, z):
	global UDvar, LRvar
	if abs(x) > UD_TOLERANCE:
		eUD = x * KpUD
		if eUD > SPEED_UD_MAX:
			eUD = SPEED_UD_MAX
		if eUD < -SPEED_UD_MAX:
			eUD = -SPEED_UD_MAX

		if eUD > 0:
			if vehicle.rangefinder.distance > MIN_HEIGHT:
				vehicle.channels.overrides[THRUST_CHANNEL] = 1500 - eUD
				UDvar =  1500 - eUD
			else:
				print("Minimum height reached")
				print(vehicle.rangefinder.distance)
				vehicle.channels.overrides[THRUST_CHANNEL] = 1500
				UDvar = 1500
		else:
			if vehicle.rangefinder.distance < MAX_HEIGHT:
				vehicle.channels.overrides[THRUST_CHANNEL] = 1500 - eUD
				UDvar = 1500 - eUD


			else:
				print("Maximim height reached")
				print(vehicle.rangefinder.distance)
				vehicle.channels.overrides[THRUST_CHANNEL] = 1500
				UDvar = 1500

	else:
		vehicle.channels.overrides[THRUST_CHANNEL] = 1500
		UDvar = 1500

	# y left right, y plus go right
	if abs(y) > LR_TOLERANCE:
		eLR = y * KpLR
		if eLR > SPEED_LR_MAX:
			eLR = SPEED_LR_MAX
		if eLR < -SPEED_LR_MAX:
			eLR = -SPEED_LR_MAX
		vehicle.channels.overrides[LR_CHANNEL] = 1500 + eLR
		LRvar = 1500 + eLR
	else:
		vehicle.channels.overrides[LR_CHANNEL] = 1500
		LRvar = 1500

	eBF = z - BF_TARGET
	if abs(eBF) > BF_TOLERANCE:
		eBF = eBF * KpBF
		if eBF > SPEED_BF_MAX:
			eBF = SPEED_BF_MAX
		if eBF < -SPEED_BF_MAX:
			eBF = -SPEED_BF_MAX
		eBF = 0
		vehicle.channels.overrides[BF_CHANNEL] = 1500 - eBF
	else:
		vehicle.channels.overrides[BF_CHANNEL] = 1500


#######################################################################
#
# MAIN
#
#######################################################################

if use_px==1:
	vehicle = dronekit.connect(connection_string, wait_ready=True, timeout=120)

	print("Check altitude")
	while vehicle.rangefinder.distance> 3:
		print(vehicle.rangefinder.distance)
		time.sleep(0.5)
	# Ensure the load is locked
	# drop_target(0)

	# Set vehicle mode
	desired_mode = 'LOITER'
	while vehicle.mode != desired_mode:
		vehicle.mode = dronekit.VehicleMode(desired_mode)
		time.sleep(0.5)

# print("Waitting the drone is armed")
# while not vehicle.armed:
#	time.sleep(0.5)
# print("Drone is ARMED")


clost = 0


if __name__ == '__main__':
	rospy.init_node('drone_tf_listener')
	listener = tf.TransformListener()


	if use_px==1:
		print("Waitting the drone is armed")
		while not vehicle.armed:
			time.sleep(0.5)

		# Waitting 2 second for situational awareness
		time.sleep(1)


		print("Taking off")
		while True:
			vehicle.channels.overrides[THRUST_CHANNEL] = TAKE_OFF_THRUST
			if not vehicle.armed:
				break

			if vehicle.rangefinder.distance >= TARGET_ALTITUDE:
				print('Reached target altitude: {0:.2f}m'.format(vehicle.rangefinder.distance))
				break
			else:
				print("Altitude: {0:.2f}m".format(vehicle.rangefinder.distance))

			if vehicle.rangefinder.distance >= MAX_TARGET_ALTITUDE:
				break
			time.sleep(0.1)

		# Altitude reached hover for 2 seconds
		if vehicle.rangefinder.distance < MAX_TARGET_ALTITUDE:
			hover(2)


	state = 10
	counterFront = 0
	print("Main loop")
	rate = rospy.Rate(20.0)  # type: Rate

	tVTOL1_now = 0
	tVTOL1_prev = 0
	tVTOL2_now = 0
	tVTOL2_prev = 0
	tVTOL3_now = 0
	tVTOL3_prev = 0
	while not rospy.is_shutdown():

		tVTOL1_prev = tVTOL1_now
		tVTOL2_prev = tVTOL2_now
		tVTOL3_prev = tVTOL3_now


		try:
			(transVTOL1, rotVTOL1) = listener.lookupTransform('/drone', '/VTOL1', rospy.Time(0))
			tVTOL1_now = listener.getLatestCommonTime("/drone", "/VTOL1")
		except:
			dm = 1

		try:
			(transVTOL2, rotVTOL2) = listener.lookupTransform('/drone', '/VTOL2', rospy.Time(0))
			tVTOL2_now = listener.getLatestCommonTime("/drone", "/VTOL2")
		except:
			dm = 1

		try:
			(transVTOL3, rotVTOL3) = listener.lookupTransform('/drone', '/VTOL3', rospy.Time(0))
			tVTOL3_now = listener.getLatestCommonTime("/drone", "/VTOL3")
		except:
			dm = 1

		if tVTOL1_now!=tVTOL1_prev:
			VTOL1detected=1
		else:
			VTOL1detected=0

		if tVTOL2_now!=tVTOL2_prev:
			VTOL2detected=1
		else:
			VTOL2detected=0

		if tVTOL3_now!=tVTOL3_prev:
			VTOL3detected=1
		else:
			VTOL3detected=0

		if show_QR==1:
			if VTOL1detected == 1:
				print("VTOL1 {:.2f} ".format(transVTOL1[0]) + "{:.2f} ".format(transVTOL1[1]) + "{:.2f} --> ".format(
					transVTOL1[2]) + "{:.2f} ".format(rotVTOL1[0]) + "{:.2f} ".format(rotVTOL1[1]) + "{:.2f} ".format(
					rotVTOL1[2]) + "{:.2f} ".format(rotVTOL1[3]))
			else:
				print("VTOL 1 not detected")

			if VTOL2detected == 1:
				print("VTOL2 {:.2f} ".format(transVTOL2[0]) + "{:.2f} ".format(transVTOL2[1]) + "{:.2f} --> ".format(
					transVTOL2[2]) + "{:.2f} ".format(rotVTOL2[0]) + "{:.2f} ".format(rotVTOL2[1]) + "{:.2f} ".format(
					rotVTOL2[2]) + "{:.2f} ".format(rotVTOL2[3]))
			else:
				print("VTOL 2 not detected")


			if VTOL3detected == 1:
				print("VTOL3 {:.2f} ".format(transVTOL3[0]) + "{:.2f} ".format(transVTOL3[1]) + "{:.2f} --> ".format(
					transVTOL3[2]) + "{:.2f} ".format(rotVTOL3[0]) + "{:.2f} ".format(rotVTOL3[1]) + "{:.2f} ".format(
					rotVTOL3[2]) + "{:.2f} ".format(rotVTOL3[3]))
			else:
				print("VTOL 3 not detected")



		if use_px==1:

			# Protection if mode changed the exit
			vehicle.mode = dronekit.VehicleMode(desired_mode)
			if vehicle.mode != 'LOITER':
				hover_once()
				break

			if vehicle.armed==False:
				hover_once()
				break

			# Waitting until QR is detected
			if state == 10:
				print("Hovering until see QR")
				hover_once()

#				print("Going forward until see QR")
#				back_forward_once(100)
#				counterFront = counterFront + 1
#				if counterFront > FRONT_SEARCH_T * 20:
#					hover(2)
#					break

				if VTOL3detected == 1:
					print("QR VTOL 3 is detected")
					hover_once()
					state = 20
					cstabil = 0
					clost = 0

			# Correcting pose QR
			if state == 20:
				if VTOL3detected == 1:
					print("QR VTOL3 is detected")
					clost = 0

					correct_altitude(transVTOL3[1],transVTOL3[0],transVTOL3[2])
					if (vehicle.channels.overrides[1] == 1500) and (vehicle.channels.overrides[2] == 1500) and (
						vehicle.channels.overrides[3] == 1500):
						cstabil = cstabil + 1
						if cstabil > STABLE_TIME * 20:
							state = 30
							statecounter = 0

				else:
					clost = clost + 1
					if clost > 10:
						clost = 12
						hover_once()
						print("QR lost hovering")

			if state == 30:
				print("State 30")
				statecounter = statecounter + 1
				hover_once()
				# if statecounter > 2*20:
				# 	drop_target(1)
				if statecounter > 4*20:
					state = 40
					statecounter = 0

			if state == 40:
				statecounter = statecounter + 1
				back_forward_once(50)
				print("State 40")
				print(statecounter)
				if statecounter > 2*20:
					hover_once()
					break





			print("UD:{0:.2f}  LR:{0:.2f}  BF:{0:.2f}".format(UDvar,
														LRvar,
														vehicle.channels.overrides[BF_CHANNEL]))
			print(state)

		rate.sleep()

if use_px==1:
	print("Setting LAND mode...")
	vehicle.mode = dronekit.VehicleMode('LAND')
	time.sleep(10)

	# Close vehicle object before exiting script
	print("Close vehicle object")
	vehicle.close()

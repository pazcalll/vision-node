#!/usr/bin/python

import dronekit
import dronekit_sitl
from pymavlink import mavutil
import time

#from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
#from pymavlink import mavutil # Needed for command message definitions
#import time
#import math

load1dropped = 0
load2dropped = 0
load3dropped = 0

connection_string = 'udp:127.0.0.1:14550'


vehicle = dronekit.connect(connection_string, wait_ready=True, timeout=120)

def drop_target(no_target):
	if no_target == 0:
		msg = vehicle.message_factory.command_long_encode(
			0, 0,  # target_system, target_component
			mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
			0,  # confirmation
			12,  # servo number
			900,  # servo position between 1000 and 2000
			0, 0, 0, 0, 0)  # param 3 ~ 7 not used
		load1dropped = 1

	if no_target==1:
		msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
        0,  # confirmation
        12,  # servo number
        600,  # servo position between 1000 and 2000
        0, 0, 0, 0, 0)  # param 3 ~ 7 not used
		load1dropped = 1

	if no_target == 2:
		msg = vehicle.message_factory.command_long_encode(
			0, 0,  # target_system, target_component
			mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
			0,  # confirmation
			12,  # servo number
			2100,  # servo position between 1000 and 2000
			0, 0, 0, 0, 0)  # param 3 ~ 7 not used
		load2dropped = 1

	if no_target == 3:
		msg = vehicle.message_factory.command_long_encode(
			0, 0,  # target_system, target_component
			mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
			0,  # confirmation
			12,  # servo number
			2300,  # servo position between 1000 and 2000
			0, 0, 0, 0, 0)  # param 3 ~ 7 not used
		load3dropped = 1

	# send command to vehicle
	vehicle.send_mavlink(msg)


# Don't let the user try to arm until autopilot is ready
#while not vehicle.is_armable:
#  print(" Waiting for vehicle to initialise... (GPS={0}, Battery={1})".format(vehicle.gps_0, vehicle.battery))
#  time.sleep(1)

# Set vehicle mode
desired_mode = 'LOITER'
while vehicle.mode != desired_mode:
  vehicle.mode = dronekit.VehicleMode(desired_mode)
  time.sleep(0.5)

drop_target(0)
print("Waitting the drone is armed")
while not vehicle.armed:
	time.sleep(0.5)

print("Drone is ARMED")

#while not vehicle.armed:
#    print("Arming motors")
#    vehicle.armed = True
#    time.sleep(0.5)

time.sleep(1)

print("Taking off")
while True:
  vehicle.channels.overrides[3] = 1657
  if vehicle.rangefinder.distance >= 1.1:
    print('Reached target altitude: {0:.2f}m'.format(vehicle.rangefinder.distance))
    break
  else:
    print("Altitude: {0:.2f}m".format(vehicle.rangefinder.distance))
  time.sleep(0.25)

print("Hover")
i = 0
while i<5:
  vehicle.channels.overrides[3] = 1500
  vehicle.channels.overrides[1] = 1500
  vehicle.channels.overrides[2] = 1500
  time.sleep(0.25)
  i = i+1



print("Front")
i = 0
while i<10:
  vehicle.channels.overrides[3] = 1500
  vehicle.channels.overrides[1] = 1500
  vehicle.channels.overrides[2] = 1400
  time.sleep(0.25)
  print('Counter: {0:.1f}'.format(i))
  i = i+1

print("Hover")
i = 0
while i<5:
  vehicle.channels.overrides[3] = 1500
  vehicle.channels.overrides[1] = 1500
  vehicle.channels.overrides[2] = 1500
  time.sleep(0.25)
  i = i+1

drop_target(2)

print("Back")
i = 0
while i<10:
  vehicle.channels.overrides[3] = 1500
  vehicle.channels.overrides[1] = 1500
  vehicle.channels.overrides[2] = 1600
  time.sleep(0.25)
  i = i+1
  print('Counter: {0:.1f}'.format(i))

print("Hover")
i = 0
while i<10:
  vehicle.channels.overrides[3] = 1500
  vehicle.channels.overrides[1] = 1500
  vehicle.channels.overrides[2] = 1500
  time.sleep(0.25)
  i = i+1

drop_target(0)


print("Setting LAND mode...")
vehicle.mode = dronekit.VehicleMode('LAND')
time.sleep(10)

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()



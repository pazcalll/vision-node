# import dronekit_sitl, time
# from dronekit import connect, VehicleMode, LocationGlobalRelative
#
# # methods
# def armtakeoff(altitude):
#     # will takeoff with current altitude
#     print ('trying to arm...')
#     # trying to connect with
#     while vehicle.is_armable is False:
#         print ('retrying')
#         time.sleep(1)
#     if vehicle.is_armable is True:
#         vehicle.mode = VehicleMode('GUIDED')
#         vehicle.armed = True
#         print('vehicle armed')
# 	vehicle.simple_takeoff(altitude)
# 	while True:
# 		altitude = vehicle.location.global_relative_frame.alt
#
# 		if altitude >= altitude-1:
# 			print('altitude reached')
# 			break
# 		time.sleep(5)
#
# # initialization
# sitl = dronekit_sitl.start_default()
#
# # connecting via udp port
# print ("connecting to vehicle...")
# vehicle = connect('127.0.0.1:14551', wait_ready=True)
# # vehicle = connect('/dev/ttyACM0', wait_ready=True)
# # vehicle = connect('/dev/ttyUSB0', wait_ready=True)
# # -------------------------------------------------------------------------------------------------
# vehicle.airspeed=5
#
# #  vehicle states
# print('vehicle is connected')
# print ('is armable: %s' % vehicle.is_armable)
# print ('GPS : %s' % vehicle.gps_0)
# print('mode :  %s' % vehicle.mode.name)
# print ('home : %s'%vehicle.home_location)
#
# # in action
# armtakeoff(5)
# print('mission success')
# print('landing...')
#
# # closing
# vehicle.close()
from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative


# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
#vehicle = connect('127.0.0.1:14551', wait_ready=True)
vehicle = connect('/dev/ACM0', wait_ready=True, baud=57600, timeout=60)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print(vehicle.location.global_relative_frame.alt)
            break
        time.sleep(1)


arm_and_takeoff(10)

print("Set default/target airspeed to 3")
vehicle.airspeed = 3

print("Going towards first point for 30 seconds ...")
point1 = LocationGlobalRelative(-7.947384, 112.616317, 10)
vehicle.simple_goto(point1)

# sleep so we can see the change in map
time.sleep(30)

print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
point2 = LocationGlobalRelative(-7.947581, 112.616094, 20)
vehicle.simple_goto(point2, groundspeed=10)

# sleep so we can see the change in map
time.sleep(30)

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop(vehicle)

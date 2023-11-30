import time

from dronekit import *
from pymavlink import mavutil

vehicle = None 

vehicle = connect('/dev/ttyTHS1', wait_ready=False, baud=57600)
print("drone connected")

time.sleep(2)
#STABILIZE
vehicle.mode    = VehicleMode("ALT_HOLD")
vehicle.armed   = True

# Confirm vehicle armed before attempting to take off
while not vehicle.armed:
	print (" Waiting for arming...")
	time.sleep(1)

test = vehicle.battery
print(test)

test2 = vehicle.location.global_frame
print(test2)

if vehicle.mode.name == "ALT_HOLD":
	print("1234")



time.sleep(2)

vehicle.mode    = VehicleMode("STABILIZE")
print("STABILIZE")
print("Flight mode: " + str(vehicle.mode.name))
time.sleep(5)
vehicle.arm = False

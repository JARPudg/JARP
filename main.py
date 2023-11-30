import time, sys
import cv2

# Import PixHawk libraries
from dronekit import *
from pymavlink import mavutil

# import Yolov8
from ultralytics import YOLO

# Load the YOLOv8n model
model = YOLO('yolov8n.pt')

# Config
alt = 2 # Altitude 
height = 2
speed = 3 #m/s
vehicle = None 

# PID values
P = 0.2
I = 0.1
D = 0.1
prev_error = 0
integral = 0

#Frame data
lenght = 512
width = 384

# Conect PixHawk with serial port to Jetson
vehicle = connect('/dev/ttyTHS1', wait_ready=False, baud=57600)
print("drone connected")

def send_movement_command_YAW(heading):
	speed = 0 
	direction = 1 #direction -1 ccw, 1 cw
    
	#heading 0 to 360 degree. if negative then ccw 
    
	print("Sending YAW movement command with heading: %f" % heading)

	if heading < 0:
		heading = heading*-1
		direction = -1

	#point drone into correct heading 
	msg = vehicle.message_factory.command_long_encode(
		0, 0,       
		mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
		0,          
		heading,    
		speed,      #speed deg/s
		direction,  
		1,          #relative offset 1
		0, 0, 0)    

	# send command to vehicle
	vehicle.send_mavlink(msg)
    
def land():
	print("Setting LAND mode...")
	vehicle.mode = VehicleMode("LAND")
	sys.exit(0)
    
def arm_and_takeoff(aTargetAltitude):

	#set default groundspeed low for safety 
	print ("setting groundspeed to 3")
	vehicle.groundspeed = 3

	print ("Basic pre-arm checks")
	# Don't try to arm until autopilot is ready
	while not vehicle.is_armable:
		print (" Waiting for vehicle to initialise...")
		time.sleep(1)

	print ("Arming motors")
	# Copter should arm in GUIDED mode
	vehicle.mode    = VehicleMode("GUIDED")
	vehicle.armed   = True

	# Confirm vehicle armed before attempting to take off
	while not vehicle.armed:
		print (" Waiting for arming...")
		time.sleep(1)

	print ("Taking off!")
	vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

	# Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
	#  after Vehicle.simple_takeoff will execute immediately).
	while True:
		print (" Altitude: ", vehicle.location.global_relative_frame.alt)
		#Break and return from function just below target altitude.
		if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
			print ("Reached target altitude")
			break
			time.sleep(1)
			
def get_mode():
	return vehicle.mode.name
    

# Open the video file
video_path = "/dev/video0"
cap = cv2.VideoCapture(video_path)

while cap.isOpened():
# Read a frame from the video
	success, frame = cap.read()

	if success:
		
		# Arm the drone and takeoff for a altitude
		arm_and_takeoff(alt)
		 
		# stop_drone()
		
		# Run YOLOv8 tracking on the frame, persisting tracks between frames
		results = model.track(frame, persist=True, classes=11, imgsz=512)

		# Boxes is the center of the detection
		boxes = results[0].boxes.xywh
		
		for box in boxes:
			x, y, w, h = box
			error = 256 - x
			integral = integral + error
			derivative = error - prev_error
			output = P*error + I*integral + D*derivative
			x_out = ((outout/512)*2)-1
			send_movement_command_YAW(out*45)
			prev_error = error
			
		# Visualize the results on the frame
		#annotated_frame = results[0].plot()

		# Display the annotated frame
		#cv2.imshow("YOLOv8 Tracking", annotated_frame)

		# Break the loop if 'q' is pressed
		if cv2.waitKey(1) & 0xFF == ord("q"):
			break
		   
		if get_mode() == "ALT_HOLD"
			#vehicle.mode = VehicleMode("")
			break
	else:
		# Break the loop if the end of the video is reached
		break

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()

	



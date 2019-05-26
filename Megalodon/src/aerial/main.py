from time import sleep
from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import parse.py
import consts.py

def arm():
	#Arms drone. Does it need to be its own function? Idek.
	while not vehicle.is_armable:
		print("Waiting for drone to initialize...")
		sleep(1)
	print("Initialized. *Most* systems go.")
	vehicle.armed = True

def takeoff_and_planned_flight():
	#Commands drone to follow planned_flight.plan.
	cmds = vehicle.commands
	cmds.clear()
	for i in parse("planned_flight.plan"):
		cmds.add(Command(*i))
	cmds.upload()
	vehicle.mode = VehicleMode("AUTO")
	#When on the ground, Copter 3.3+ must receive a MAV_CMD_MISSION_START command before taking off. Yay Mavlink!
	mavcmd = vehicle.message_factory.command_long_encode(
		0, 0,
		mavutil.mavlink.MAV_CMD_MISSION_START, 0,
		0, 0, 0, 0, 0, 0, 0
	)
	vehicle.send_mavlink(mavcmd)
	#As of 5-19-19, the drone plops itself in the water at the end of planned_mission.plan. Might want to change that?
	#Note: main script should wait for vehicle.armed to become False, then proceed with other mission stuff.
	
def takeoff_and_home():
	#todo. After floating to water's surface, drone takes off and returns to home base.

def takeoff_and_search():
	#todo. After floating to water's surface, drone takes off, searches for tag, centers itself over tag, and descends.
	#Btw! Do some research into qgc zigzag flight patterns for search.
	#Also requires the apriltag function.

print("Connecting to drone...")	
vehicle = connect(consts["connection_string"], wait_ready = True)
print("Connected.")
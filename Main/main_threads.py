
import pygame
import time
import serial
import threading
import queue
import math
import os
os.environ['SDL_JOYSTICK_HIDAPI_PS4_RUMBLE'] = '1' #this enables vibrations of PS4 controller


from serial_comunication import read_and_wait, wait_for_DONE, serial_comunication_loop, set_joints
from Robot_classes import Robot
from joystick_functions import initialize_joystick, get_joystick


def joystickToDeltas_bisturi(axes, buttons, robot, shared_queue, joystick):


	""" This function translates joystick inputs to robot change in positions. It is specific to the bisturi robot.
	It should then convert robot position to change in joint values with an inverse kinematics function
	In the end it call robot.move_joints which should send the new joint values to the shared queue
	
	The other thread (serial comunication thread) acesses this shared queue and makes the robot move in accordance with the new joint values 
	"""
	if buttons[3]==1:
		bool_if_changed, sensitivity = robot.set_sensitivity(True) #Change the sensitivity of the robot (between high and low sensitivity) - if triangle is pressed

	else:
		robot.set_sensitivity(False) #inform robot that triangle button has been depressed
		bool_if_changed=False


	if  bool_if_changed: #this vibrates the controller to let the user know that the sensitivity has  been changed 
		if sensitivity== 'High':
			joystick.rumble(0, 0.8, 100)
		elif sensitivity== 'Low': #Depending on whether sensitivity has become high or low, it vibrates with high or low frequency
			joystick.rumble(0.8, 0, 100)

	sensitivity = robot.get_sensitivity()
	x = axes[0]*sensitivity     #x axis controled by horizontal movemento of left analogue
	y = -axes[1]*sensitivity     #y axis controled by vertical movemento of left analogue
	pitch = axes[2]*sensitivity # pitch axis controled by horizontal movement of right analogue
	roll = axes[3]*sensitivity  #roll axis controled by vertical movement of right analogue

	#L2 and R2 triggers to control z position (L2 and R2 are equal to -1 when not pressed. And equal to 1 when pressed)
	if axes[4]!= -1 and axes[5]== -1: #L2 pressed and R2 not pressed - robot descending
		z= -sensitivity*(axes[2] + 1) /2
	elif axes[5]!= -1 and axes[4]== -1:#R2 pressed and L2 not pressed - robot ascending
		z= sensitivity*(axes[5] + 1) /2  
	else:
		z=0
	
	#In here, there should be an inverse kinematics function to translate positions to joints

	robot.move_joints([x,y,z,pitch,roll], shared_queue)
	#robot.move_pos([x,y,z,pitch,roll], shared_queue) #Try in LAB---

	 

def move_2planes_semicircle(pos,x, c, v):
	#This function  is used to create the aprropriate trajectory for the camera robot. The trajectory along a specified surface.
	#In this case, the surface is 2 perpenbdicular planes, united by a 1/4 cilindrical surface.
	 

	if v<(2*c/3):
		z=v
		y=0
		p=math.pi/2
	elif v<((2/3) + (math.pi/6))*c:
		theta = p = (v-(2*c/3))/3
		y= c*math.sin(theta) #theta in radians
		z= c* (1 - math.cos(theta)/3)
	else:
		z=c
		p=0
		y=v - (c*((1/3) + (math.pi/6)))
	
	new_pos=[x,y,z,p,0]
	delta_pos = [new_pos[i] - pos[i] for i in range(len(new_pos))]

	return new_pos, delta_pos 
 

def joystickToDeltas_camera(axes, buttons, robot, shared_queue):
	
	#still in development
	""" This function translates joystick inputs to robot change in positions. It is specific to the camera robot.
	It uses move_2planes_semicircle function to help define the robot position

	It should then convert robot position to change in joint values with an inverse kinematics funcion
	In the end it call robot.move_joints which should send the new joint values to the shared queue
	
	The other thread (serial comunication thread) acesses this shared queue and makes the robot move in accordance with the new joint values 
	
 	"""
	sensitivity = robot.get_sensitivity()
	x = (-buttons[13] + buttons[14])*sensitivity		#change x(horiozntal position) with left and right arrows
	v_delta= (-buttons[12] + buttons[11])*sensitivity 	#change v with up and down arrows
	c_delta= (-buttons[9] + buttons[10])*sensitivity 	#change c constant with L1 and R1
	
	pos=robot.get_pos()
	robot.change_c(c_delta)
	c= robot.get_c()
	robot.change_v(v_delta) 
	v= robot.get_v() #v will be a value between 0 and (4/3 + pii/6)*c

	new_pos, delta_pos = move_2planes_semicircle(pos,x, c, v)	

	#In here, there should be an inverse kinematics function to translate positions to joints

	robot.move_pos(new_pos, delta_pos , shared_queue)

	



def moverobots(axes, buttons, robots, shared_queue, joystick):
	#this function should: translate joystick movements to actual movements for the robots using inverse and foward kinematics and apply the movement to the robot's class function move(self)
	#eventually it can also check for colision
	for i, robot in enumerate(robots):
		if i ==0:
			joystickToDeltas_bisturi(axes, buttons,  robot, shared_queue, joystick)  #i=0 correspondos to robot 0 - robot with bisturi
		elif i==1:
			joystickToDeltas_camera(axes, buttons,  robot, shared_queue)  #i=1 correspondos to robot 1 - robot with camera

def joystick_loop(shared_queue, robots):
	# Counter 
	count = 0
	joystickLoopClock = pygame.time.Clock()
	joystick = initialize_joystick()
	FPS=10
	
	try:
		while True:
		# Handle events
			if pygame.event.peek(): #if there are events waiting in joystick queue
				axes, buttons, quit = get_joystick(joystick)
			if quit:
				return
			#print('Joystick values: \n', axes)
			count += 1
			
			moverobots(axes, buttons, robots, shared_queue, joystick) #this function should: translate joystick movements to actual movements for the robots using inverse and foward kinematics and update the shared_queue with new joint values
			
			joystickLoopClock.tick(FPS) # do not run loop faster than n times a second

	except KeyboardInterrupt:
		pass
	finally:
		# Clean up
		pygame.quit()
		for robot in robots:
			robot.housekeeping()


	
def main():
	bisturi_robot=Robot('COM4',5, -200, 5, 'bisturi') #robot speed and sensitivity values high and low for robot movement
	robots=[bisturi_robot] #eventually this list will have both the bisturi and camera robot
	serBisturi=bisturi_robot.get_serial()
	shared_queue = queue.Queue() #this queue will save values for the robot's joint deltas

	joystick_thread = threading.Thread(target=joystick_loop, args=(shared_queue,robots))
	bisturi_serial_thread = threading.Thread(target=serial_comunication_loop, args=(shared_queue, serBisturi))
	
	joystick_thread.start()
	bisturi_serial_thread.start()
	
	joystick_thread.join() #this ensures that the main function only stops when the joystick loop thread is done running. This function should only end after the housekeeping of the robots
	bisturi_serial_thread.join()


if __name__ == "__main__":
    main()
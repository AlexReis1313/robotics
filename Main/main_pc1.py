import pygame
import time
import serial
import threading
import queue
import math
import socket
import numpy as np


from joystick_functions import *
from robot_classes_manual import *
from communication_client import *

def do_obstacle_avoidance(bisturi_pose, camera_pose, L, info_computer_share):
	safety_distance = 100
	
	transformation_matrix = np.array([
		[-1, 0, 0, L],
		[0, -1, 0, 0],
		[0, 0, 1, 0],
		[0, 0, 0, 1]
	])

	# don't exactly know how to get the positions, for now let's say the inputs are given
	end_effector1_R1 = np.concatenate([bisturi_pose[5:8], np.array([1])])
	end_effector2_R2 = np.concatenate([camera_pose[5:8], np.array([1])])
	end_effector2_R1 = np.dot(transformation_matrix, end_effector2_R2)
	
	# Calculate the Euclidean distance between the two points
	distance = np.linalg.norm(end_effector1_R1 - end_effector2_R1)
	
	# Check for collision and return result
	if distance > safety_distance:
		info_computer_share['colision']=False
		return False  # No collision
	else:
		info_computer_share['colision']=True
		return True  # Collision

def camera_robot_loop(FPS, athomeBool,joystick_queue, shared_camera_pos):
	clock = pygame.time.Clock()
	camera_robot = cameraRobot(shared_camera_pos, comPort='COM3', atHome=athomeBool)
	axes=[0]*4+[-1,-1]
	buttons=[0]*15
	try:
		while True:
		# Handle events
			quit=False
			if not joystick_queue.empty(): #if there are events waiting in joystick queue
				joystick_values = joystick_queue.get()
				#print('Joystick at camera',joystick_values)
				axes, buttons, quit, coliding = joystick_values[:6], joystick_values[6:-2], joystick_values[-2], joystick_values[-1]
				while not joystick_queue.empty():
					joystick_queue.get()

			if quit:
				return
			if not coliding:
				camera_robot.move(axes, buttons)
			clock.tick(FPS)
	
	except KeyboardInterrupt:
		pass
	finally:
		camera_robot.housekeeping() #this ends the manual mode and closes the serial port


def bisturi_robot_controll_loop( FPS, L, athomeBool,joystick_queue, shared_camera_pos, info_computer_share):
	joystick = initialize_joystick()
	clock = pygame.time.Clock()
	bisturi_robot=Robot(joystick, info_computer_share ,comPort='COM4', atHome=athomeBool)
	count=0
	colision =False
	try:
		while True:
		# Handle events
			quit=False
			if pygame.event.peek(): #if there are events waiting in joystick queue
				axes, buttons, quit = get_joystick(joystick)
				joystick_queue.put(axes + buttons + [quit]+[False]) #last false is for colision

			if quit:
				joystick_queue.put(axes + buttons + [True]+[False])
				info_computer_share['state']=4
				return
			""" if count> 5*FPS: #happens one time each second
				count=0
				bisturi_robot.manual_end()
				bisturi_robot.calculate_pos()
				bisturi_robot.update_bisturi_pos_shared()
				bisturi_robot.manual_start_midle() """

			count+=1
		
			if not athomeBool:
				colision = do_obstacle_avoidance(bisturi_pose = bisturi_robot.get_last_pos(), camera_pose=shared_camera_pos, L=L, info_computer_share=info_computer_share)
			if not colision:
				bisturi_robot.iterate(axes,buttons)
			else:
				joystick_queue.put(axes + buttons + [quit]+[True])

			clock.tick(FPS)
	
	except KeyboardInterrupt:
		joystick_queue.put(axes + buttons + [True]+[False])
		pass
	finally:
		pygame.quit()
		bisturi_robot.housekeeping() #this ends the manual mode and closes the serial port




def send_robot_data(FPS,info_computer_share):
	FPS = 1
	#HEADER, FORMAT, DISCONNECT_MESSAGE, ADDR = define_constants()
	#connect_to_server(ADDR, HEADER, FORMAT, DISCONNECT_MESSAGE,FPS, info_computer_share)
	while info_computer_share['state'] != 4:
		print('info_computer_share', str(info_computer_share))
		time.sleep(1)
def main():
	FPS=40
	L= 1000 #distance between base of the 2 robots
	athomeBool=True
	joystick_queue = queue.LifoQueue() #this queue will save values for the joystick's current state - it will be shared between the loops for both robots
	shared_camera_pos =[0,0,0,0,0]
	info_computer_share = {'state': -1, 'last_bisturi_pos': [0,0,0,0,0],  'cutting_plan':[0,0],'coliding':False}
							#state: -1 - Initializing
							#		0 - Running joints mode
							#		1 - Running xyz mode
							#		2 - Preparing for cut
							#		3 - Cutting
							#		4 - Finished running
	robot_bisturi_thread = threading.Thread(target=bisturi_robot_controll_loop, args=(FPS,L, athomeBool, joystick_queue, shared_camera_pos, info_computer_share))
	robot_camera_thread = threading.Thread(target=camera_robot_loop, args=(FPS,athomeBool, joystick_queue, shared_camera_pos))
	send_data_thread = threading.Thread(target=send_robot_data,args=(FPS,info_computer_share))
	
	robot_bisturi_thread.start()
	robot_camera_thread.start()
	send_data_thread.start()
	
	robot_bisturi_thread.join() #this ensures that the main function only stops when the joystick loop thread is done running. This function should only end after the housekeeping of the robots
	robot_camera_thread.join()
	send_data_thread.join()

if __name__ == "__main__":
    main()
	
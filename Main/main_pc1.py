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

def do_obstacle_avoidance(bisturi_pose, camera_pose, L):
    safety_distance = 10
    
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
        return 1  # No collision
    else:
        return -1  # Collision

def camera_robot_loop(athomeBool,joystick_queue, shared_camera_pos):
	FPS=40
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
				axes, buttons, quit = joystick_values[:6], joystick_values[6:-1], joystick_values[-1]
				while not joystick_queue.empty():
					joystick_queue.get()

			if quit:
				return
			camera_robot.move(axes, buttons)
			clock.tick(FPS)
	
	except KeyboardInterrupt:
		pass
	finally:
		camera_robot.housekeeping() #this ends the manual mode and closes the serial port


def bisturi_robot_controll_loop(athomeBool,joystick_queue, shared_camera_pos, info_computer_share):
	joystick = initialize_joystick()
	FPS=40
	clock = pygame.time.Clock()
	bisturi_robot=Robot(joystick,FPS, info_computer_share ,comPort='COM4', atHome=False)
	count=0
	try:
		while True:
		# Handle events
			quit=False
			if pygame.event.peek(): #if there are events waiting in joystick queue
				axes, buttons, quit = get_joystick(joystick)
				joystick_queue.put(axes + buttons + [quit])

			if quit or  bisturi_robot.get_stop_program():
				joystick_queue.put(axes + buttons + [True])
				info_computer_share['state']=4
				return
			""" if count> 5*FPS: #happens one time each second
				count=0
				bisturi_robot.manual_end()
				bisturi_robot.calculate_pos()
				bisturi_robot.update_bisturi_pos_shared()
				bisturi_robot.manual_start_midle() """

			count+=1
			#do_obstacle_avoidance(bisturi_robot, shared_camera_pos, bisturi_pose = bisturi_robot.get_last_pos() , calibration_matrix = info_computer_share['calibration_matrix']), 
			bisturi_robot.iterate(axes,buttons)
			clock.tick(FPS)
	
	except KeyboardInterrupt:
		joystick_queue.put(axes + buttons + [True])
		pass
	finally:
		pygame.quit()
		bisturi_robot.housekeeping() #this ends the manual mode and closes the serial port


# def computer_comunication_loop(info_computer_share):
# 	FPS=10
# 	client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# 	client.connect(('194.210.177.59', 50000))
# 	clock = pygame.time.Clock()
# 	last_state = info_computer_share['state']
# 	last_bisturi_pos=info_computer_share['last_bisturi_pos']
# 	last_cutting_plan = info_computer_share['cutting_plan']
# 	k=True
# 	while k:
# 			data = client.recv(1024)
# 			if not data:
# 				k=False

# 			elif data.decode()!='':#something was received
# 				received = data.decode().split('\n')
# 				#update info_computer_share parameters - like calibration matrix

# 			else:#send data
# 				message=''
# 				if last_state != info_computer_share['state']:
# 					last_state = info_computer_share['state']
# 					message +=f'New State: {last_state}\n'

# 				if last_bisturi_pos !=info_computer_share['last_bisturi_pos']:
# 					last_bisturi_pos=info_computer_share['last_bisturi_pos']
# 					message +=f'Bisturi Pose: {last_bisturi_pos}\n'

# 				if 	last_cutting_plan != info_computer_share['cutting_plan']:
# 					last_cutting_plan = info_computer_share['cutting_plan']
# 					message +=f'Cutting Plan: {last_cutting_plan}\n'
				
				

# 				len_mssg=len(message)
# 				if len_mssg:
# 					message+=f'{len_mssg}\n'#length of carachters to make sure the other computer has received the correct info

# 				client.send(message.encode('utf-8'))

# 				while True:#loop to check if the other computer received all the info. If it did not, resend information
# 					check_received = client.recv(1024)
# 					if bool(check_received.decode()):
# 						break
# 					else:
# 						client.send(message.encode('utf-8'))


# 			clock.tick(FPS)

def send_robot_data(info_computer_share):
	FPS = 1
	HEADER, FORMAT, DISCONNECT_MESSAGE, ADDR = define_constants()
	connect_to_server(ADDR, HEADER, FORMAT, DISCONNECT_MESSAGE,FPS, info_computer_share)

def main():
	athomeBool=True
	joystick_queue = queue.LifoQueue() #this queue will save values for the joystick's current state - it will be shared between the loops for both robots
	shared_camera_pos =[0,0,0,0,0]
	info_computer_share = {'state': -1, 'last_bisturi_pos': [0,0,0,0,0], 'calibration_matrix': None, 'cutting_plan':[0,0]}
							#state: -1 if in no state
							#		0 if in calibration
							#		1 if in running
							#		2 if preparing for cut
							#		3 doing cut
							#		4 finished running
	robot_bisturi_thread = threading.Thread(target=bisturi_robot_controll_loop, args=(athomeBool, joystick_queue, shared_camera_pos, info_computer_share))
	robot_camera_thread = threading.Thread(target=camera_robot_loop, args=(athomeBool, joystick_queue, shared_camera_pos))
	#send_data_thread = threading.Thread(target=send_robot_data,args=(info_computer_share))
	
	robot_bisturi_thread.start()
	robot_camera_thread.start()
	#computer_comunication_thread.start()
	
	robot_bisturi_thread.join() #this ensures that the main function only stops when the joystick loop thread is done running. This function should only end after the housekeeping of the robots
	robot_camera_thread.join()

if __name__ == "__main__":
    main()
	
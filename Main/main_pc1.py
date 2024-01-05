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
from colision_detection import *

def camera_robot_loop(FPS, athomeBool,joystick_queue, shared_camera_pos):
	clock = pygame.time.Clock()
	camera_robot = cameraRobot(shared_camera_pos, comPort='COM7', atHome=athomeBool)
	axes=[0]*4+[-1,-1]
	buttons=[0]*15
	coliding=False
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
	bisturi_robot=Robot(joystick, info_computer_share ,comPort='COM5', atHome=athomeBool)
	colision_count=0
	last_camera_pose=[0,0,0,0,0]
	colision =False
	aux=False
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
			
			if bisturi_robot.getcount()> 7*FPS: #happens if more than 7 seconds have passed since last time a LISTPV was done
				null_axes=[0,0,0,0,-1,-1]
				check_axes=[abs(axes[i]-null_axes[i]) for i in range(len(axes))]
				if max(check_axes)<0.2 and buttons[9]==0 and buttons[10]==0 : #if no button is being pressed, get position of robot
					
					bisturi_robot.manual_end()
					time.sleep(0.5)
					bisturi_robot.calculate_pos() #this returns count to zero
					time.sleep(0.5)
					bisturi_robot.update_bisturi_pos_shared()
					bisturi_robot.manual_start_midle()
					time.sleep(0.3)
			

			""" if buttons[15] and not aux:
				aux=True
				colision=True
				info_computer_share['coliding']=True """
			
			if not shared_camera_pos.empty(): #if there are events waiting in joystick queue
				last_camera_pose = shared_camera_pos.get()
				#print('Joystick at camera',joystick_values)
				while not shared_camera_pos.empty():
					shared_camera_pos.get()
			
			colision = do_obstacle_avoidance(bisturi_pose = bisturi_robot.get_last_pos(), camera_pose=last_camera_pose, L=L, info_computer_share=info_computer_share)
			if not colision:
				bisturi_robot.iterate(axes,buttons)
				colision_count=0

			elif colision_count>8*FPS:
				bisturi_robot.iterate(axes,buttons)	

			else:
				joystick_queue.put(axes + buttons + [quit]+[True])		
				colision_count+=1

			clock.tick(FPS)
	
	except KeyboardInterrupt:
		joystick_queue.put(axes + buttons + [True]+[False])
		pass
	finally:
		pygame.quit()
		bisturi_robot.housekeeping() #this ends the manual mode and closes the serial port




def send_robot_data(athome,info_computer_share):
	if not athome:
		FPS = 5
		HEADER, FORMAT, DISCONNECT_MESSAGE, ADDR = define_constants()
		connect_to_server(ADDR, HEADER, FORMAT, DISCONNECT_MESSAGE,FPS, info_computer_share)
	else:
		while info_computer_share['state'] != 4:
			print('info_computer_share', str(info_computer_share))
			time.sleep(1)
def main():
	FPS=40
	L= 1200 #distance between base of the 2 robots in mm
	L=int(input('What is the distance between the center of the base of the 2 robots? (in mm)'))
	athomeBool=False
	joystick_queue = queue.LifoQueue() #this queue will save values for the joystick's current state - it will be shared between the loops for both robots
	shared_camera_pos =queue.LifoQueue()
	info_computer_share = {'state': -1, 'last_bisturi_pos': [0,0,0,0,0],  'cutting_plan':[0,0],'coliding':False}
							#state: -1 - Initializing
							#		0 - Running joints mode
							#		1 - Running xyz mode
							#		2 - Preparing for cut
							#		3 - Cutting
							#		4 - Finished running
	robot_bisturi_thread = threading.Thread(target=bisturi_robot_controll_loop, args=(FPS,L, athomeBool, joystick_queue, shared_camera_pos, info_computer_share))
	robot_camera_thread = threading.Thread(target=camera_robot_loop, args=(FPS,athomeBool, joystick_queue, shared_camera_pos))
	send_data_thread = threading.Thread(target=send_robot_data,args=(athomeBool,info_computer_share))
	
	robot_bisturi_thread.start()
	robot_camera_thread.start()
	send_data_thread.start()
	
	robot_bisturi_thread.join() #this ensures that the main function only stops when the joystick loop thread is done running. This function should only end after the housekeeping of the robots
	robot_camera_thread.join()
	send_data_thread.join()

if __name__ == "__main__":
    main()
	
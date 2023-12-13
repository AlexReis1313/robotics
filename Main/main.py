import pygame
import time
import serial
import threading
import queue
import math


from joystick_functions import initialize_joystick, get_joystick
from robot_classes_manual import *






def camera_robot_loop():
	joystick = initialize_joystick()
	FPS=40
	clock = pygame.time.Clock()
	camera_robot = cameraRobot()
	try:
		while True:
		# Handle events
			if pygame.event.peek(): #if there are events waiting in joystick queue
				axes, buttons, quit = get_joystick(joystick)
			if quit:
				return
			camera_robot.move(axes, buttons)
			clock.tick(FPS)
	
	except KeyboardInterrupt:
		pass
	finally:
		pygame.quit()
		camera_robot.housekeeping() #this ends the manual mode and closes the serial port



def robot_controll_main_loop():
	sharedData=[0]
	joystick = initialize_joystick()
	FPS=40
	clock = pygame.time.Clock()
	bisturi_robot=Robot(joystick,FPS, sharedData)

	robots=[bisturi_robot]
	count=0
	try:
		while True:
		# Handle events
			if pygame.event.peek(): #if there are events waiting in joystick queue
				axes, buttons, quit = get_joystick(joystick)
			if quit or  bisturi_robot.get_stop_program():
				return
			if count> FPS: #happens one time each second
				count=0
				bisturi_robot.manual_end()
				#do_obstacle_avoidance(bisturi_robot, sharedData)
				bisturi_robot.manual_start_midle()

			#share_data_computers(buttons,bisturi_robot ,sharedData)

			count+=1
			bisturi_robot.manual_move(axes,buttons)
			clock.tick(FPS)

	
	except KeyboardInterrupt:
		pass
	finally:
		pygame.quit()
		for robot in robots:
			robot.housekeeping() #this ends the manual mode and closes the serial port



if __name__ == "__main__":
    camera_robot_loop()
	
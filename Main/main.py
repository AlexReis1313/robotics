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
	f=open('Data_robot_movement.txt','w')

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
		f.close()

		for robot in robots:
			robot.housekeeping() #this ends the manual mode and closes the serial port



def main():
	sharedData = [[0,0,0,0,0]]
	

	robot_controll_thread = threading.Thread(target=robot_controll_main_loop, args=(sharedData))
	#computer_comunication_thread = threading.Thread(target=computer_comunication_loop, args=(sharedData))
	
	robot_controll_thread.start()
	#computer_comunication_thread.start()
	
	robot_controll_thread.join() #this ensures that the main function only stops when the joystick loop thread is done running. This function should only end after the housekeeping of the robots
	#computer_comunication_thread.join()

if __name__ == "__main__":
    camera_robot_loop()
	
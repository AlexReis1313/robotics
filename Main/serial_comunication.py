import pygame
import time
import serial
import threading
import queue
import math


def read_and_wait(ser, wait_time):
#this function reads the information that the robot outputs to the computer and returns it as a string
		serString = "" # Used to hold data coming over UART
		output = ""
		flag = True
		start_time = time.time()
		while flag:
			# Wait until there is data waiting in the serial buffer
			if ser.in_waiting > 0:
				# Read data out of the buffer until a carriage return / new line is found
				serString = ser.readline()
				# Print the contents of the serial data
				try:
					output = output + serString.decode("Ascii")
					print(serString.decode("Ascii"))
				except:
					pass
			else:
				deltat = time.time() - start_time
				if deltat>wait_time:
					flag = False
		return output	

def wait_for_DONE():	#this functions reads the robot and waits for the Done. to be given
		while True:
			if 'MOVE AA \r\n\x00Done.\r\n>' == read_and_wait(0.3):
				break
		return None



def set_joints(jointDelta_values, ser, typeofcomunication_forDebug):
	deltas=jointDelta_values[0:5]
	joints=jointDelta_values[5:10]
	if typeofcomunication_forDebug==0:
		for i,joint in enumerate(joints) :
			if deltas[i] !=0:
				#printToRobot=f'SHIFT AA BY {i+1} {int(delta)} \r'
				printToRobot=f'SETPV AA {i+1} {int(joint)}\r'
				print(printToRobot)
				ser.write(printToRobot.encode('utf-8'))#change for lab
				time.sleep(0.1)
	elif typeofcomunication_forDebug==1:
		#to try in lab
		printToRobot=""
		bool=False
		for i,joint in enumerate(joints) :
			if deltas[i] !=0:
				printToRobot+=f'SETPV AA {i+1} {int(joint)} \r\n'
				bool=True
		if bool:
			print(printToRobot) #change for lab
			ser.write(printToRobot.encode('utf-8'))
			time.sleep(0.1)
	elif typeofcomunication_forDebug==2:
		#to try in lab
		printToRobot=""
		bool=False
		for i,joint in enumerate(joints) :
			if deltas[i] !=0:
				printToRobot+=f'SETPV AA {i+1} {int(joint)} \r\n\x00'
				bool=True
		if bool:
			print(printToRobot) #change for lab
			ser.write(printToRobot.encode('utf-8'))
			time.sleep(0.1)




def serial_comunication_loop(shared_queue, ser):
	typeofcomunication_forDebug=0 #alter in lab
	
	count=0
	FPS=5
	clock_serial = pygame.time.Clock()
	needs_to_move=False
	while True:
		# Check if the shared queue has data
		if not shared_queue.empty() and count<2:
			jointDelta_values=[]
			while not shared_queue.empty():
				jointDelta_values.append(shared_queue.get())            # Retrieve joint values from the shared queue
			shared_queue.task_done()
			set_joints(jointDelta_values[-1], ser,typeofcomunication_forDebug)
			needs_to_move=True
			count +=1
			

		elif needs_to_move:
			print('Moving')
			ser.write(b'MOVE AA \r')
			#read_and_wait(ser,0.5)
			time.sleep(0.5)
			needs_to_move=False
			count=0

		clock_serial.tick(FPS) # do not run loop faster than n times a second
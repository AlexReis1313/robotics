import pygame
import time
import serial
import threading
import queue
import math


def read_and_wait_original(ser, wait_time):
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

				except:
					pass
			else:
				deltat = time.time() - start_time
				if deltat>wait_time:
					flag = False
		return output	


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
					print('Output',output)
					return output	

				except:
					pass
			else:
				deltat = time.time() - start_time
				if deltat>wait_time:
					flag = False

def clean_buffer(ser):
#this function reads the information that the robot outputs to the computer and returns it as a string
		 # Used to hold data coming over UART
		
		flag = True
		while flag:
			# Wait until there is data waiting in the serial buffer
			if ser.in_waiting > 0:
				# Read data out of the buffer until a carriage return / new line is found
				ser.readline()
				# Print the contents of the serial data
				
			else:
				#print('buffer clean')
				break


def wait_for_DONE(ser, max_time):	#this functions reads the robot and waits for the Done. to be given
		start_time=time.time()
		while not str(read_and_wait(ser,0.5)).find('Done.'):
			#if 'MOVE AA \r\n\x00Done.\r\n>' == read_and_wait(0.3):
			if time.time()- start_time > max_time:
				return False
			pass
		
		#print('Done found')
		return True



def set_joints(jointDelta_values, ser, typeofcomunication_forDebug):
	deltas=jointDelta_values[0:5]
	joints=jointDelta_values[5:10]
	if typeofcomunication_forDebug==0:
		for i,joint in enumerate(joints) :
			if deltas[i] !=0:
				#printToRobot=f'SHIFT AA BY {i+1} {int(delta)} \r'
				printToRobot=f'SETPV AA {i+1} {int(joint)}\r'
				print(printToRobot)
				ser.write(printToRobot.encode('utf-8'))#change for lab ---------HERE FOR LAB
				time.sleep(0.2)

	elif typeofcomunication_forDebug==1:
		#to try in lab
		printToRobot=""
		bool=False
		for i,joint in enumerate(joints) :
			if deltas[i] !=0:
				printToRobot+=f'SETPV AA {i+1} {int(joint)} \r'
				bool=True
		if bool:
			printToPython=printToRobot.replace('\r','\n')
			print(printToPython) #change for lab
			ser.write(printToRobot.encode('utf-8')) #change for lab ---------HERE FOR LAB
			time.sleep(1)
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
			time.sleep(1)
	
	elif typeofcomunication_forDebug==3:
		#to try in lab
		printToRobot=""
		bool=False
		for i,joint in enumerate(joints) :
			if deltas[i] !=0:
				printToRobot=f'{printToRobot}SETPV AA {i+1} {int(joint)} \r'
				bool=True
		if bool:
			clean_buffer(ser)
			print(printToRobot) #change for lab
			ser.write(printToRobot.encode('utf-8'))
			wait_for_DONE(ser, 1)
		
	elif typeofcomunication_forDebug==4:
		#to try in lab
		printToRobot=""
		bool=False
		done_bool=False
		while not done_bool:
			for i,joint in enumerate(joints) :
				if deltas[i] !=0:
					printToRobot+=f'SETPV AA {i+1} {int(joint)} \r'
					bool=True
			if bool:
				clean_buffer(ser)
				print(printToRobot) #change for lab
				ser.write(printToRobot.encode('utf-8'))
				done_bool=wait_for_DONE(ser,1)
			





def set_position(position_values, ser): #Try in LAB---
	deltas=position_values[0:5]
	coordenates=position_values[5:10]
	cartesian=['X','Y', 'Z', 'P', 'R']

	for i,coordenate in enumerate(coordenates) :
		if deltas[i] !=0:
			#printToRobot=f'SHIFT AA BY {i+1} {int(delta)} \r'
			printToRobot=f'SETPVC AA {cartesian[i]} {int(coordenate)}\r'
			print(printToRobot)
			ser.write(printToRobot.encode('utf-8'))#change for lab ---------HERE FOR LAB
			time.sleep(0.3)


def serial_comunication_loop(shared_queue, ser):
	typeofcomunication_forDebug=1 #alter in lab
	
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
			#set_position(jointDelta_values[-1], ser)                   #Try in LAB---


			needs_to_move=True
			count +=1
			

		elif needs_to_move:
			print('Moving')
			clean_buffer(ser)
			ser.write(b'MOVEL AA \r') #change for lab ---------HERE FOR LAB
			clean_buffer(ser)
			time.sleep(0.3)
			wait_for_DONE(ser,10)	
			#time.sleep(1)
			needs_to_move=False
			count=0

		clock_serial.tick(FPS) # do not run loop faster than n times a second


def begin_manual(ser):

	ser.write(b'~ \r') 
	while not read_and_wait(ser, 0).find('>_'):
		pass
	ser.write(b'X\r')
	print('Manual mode is active')
	read_and_wait(ser,0.5)
	ser.write(b'S \r')
	while not read_and_wait(ser, 0).find('SPEED'):
		pass
	ser.write(b'5\r')




def manual_pos(xyzvalues, ser):
	if xyzvalues[0]>0:
		ser.write(b'1 \r')
	elif xyzvalues[0]<0:
		ser.write(b'Q \r')

	if xyzvalues[1]>0:
		ser.write(b'2 \r')

	elif xyzvalues[1]<0:
		ser.write(b'W \r')		
	
	time.sleep(1)

	if xyzvalues[0]>0:
		ser.write(b'1 \r')
	elif xyzvalues[0]<0:
		ser.write(b'Q \r')

	elif xyzvalues[1]>0:
		ser.write(b'2 \r')

	elif xyzvalues[1]<0:
		ser.write(b'W \r')





def manual_comunication_loop(shared_queue, ser):
	count=0
	FPS=10
	clock_serial = pygame.time.Clock()
	begin_manual(ser)
	while True:
		# Check if the shared queue has data
		if not shared_queue.empty():
			jointDelta_values=[]
			while not shared_queue.empty():
				jointDelta_values.append(shared_queue.get())            # Retrieve joint values from the shared queue
			shared_queue.task_done()
			manual_pos(jointDelta_values[-1], ser)
			#set_position(jointDelta_values[-1], ser)                   #Try in LAB---


			count +=1
			

		clock_serial.tick(FPS) # do not run loop faster than n times a second	

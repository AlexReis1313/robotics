import pygame
import time
import serial
import threading
import queue
import math

from serial_comunication import read_and_wait, wait_for_DONE, serial_comunication_loop, set_joints

class Robot():
	def __init__(self, comPort, robotSpeed, sensitivityHigh, sensitivityLow):
		#initialization of the robot class
		#this function opens the serial connection between the robot and the computer, defined the robot's speed % and calibrates the robot
		self.ser = serial.Serial(comPort, baudrate=9600, bytesize=8, timeout=2, parity='N', xonxoff=0)
		print("COM port in use: {0}".format(self.ser.name))
		self.ser.write(f'Speed {robotSpeed}\r'.encode('utf-8'))
		self.calibrate() 
		self.pos = self.calibratedPos
		self.sensitivity = sensitivityHigh
		self.otherSensitivity= sensitivityLow
		self.triangle_Pressed=False
		


	def get_serial(self):
		return self.ser

	def calibrate (self, robot_type):	#function that is used to calibrate the robot in the begging of running the program
		print('Move the robot arm to the calibration position. Use the Teach Pendent')
		if robot_type =='bisturi':
			input_String="Input 'y' when ready to calibrate (when the end effector is in the calibration position)"
		elif robot_type =='camera':
			input_String="Input 'y' when ready to calibrate (the end effector position should be vertically alligned with the jello)"

		while True:
			if 'y'==input(input_String):
				self.ser.write(b'HERE AA \r')
				self.calculate_pos()
				self.calibratedPos = self.get_pos()
				print('Calibration Finished')
				break
			
	def get_calibrationPos(self):#return calibration position
		#this function can be used when evaluating if the robot is going to colide with something. by returning the initial calibration position
		#for example, to check if a new position of the robot will make it colide with the table, we can check if the z axis of that position if some cm lower than the calibration positon z axis
		return self.calibratedPos
	
	def get_sensitivity(self):
		return self.sensitivity
	
	def set_sensitivity(self, buttonPressed):
		if self.triangle_Pressed and buttonPressed: #triangle was pressed and still is - no changes to sensitivity
			return None
		elif not self.triangle_Pressed and buttonPressed: #triangle was not pressed and now is pressed - change sensitivity from Low to high or High to low
			self.sensitivity , self.otherSensitivity  = self.otherSensitivity,  self.sensitivity
			self.triangle_Pressed=True
			
			auxiliar= bool(self.sensitivity >self.otherSensitivity )
			print('Sensitivity changed')
			if auxiliar:
				print('high')
			else:
				print('low')

		else: 
			self.triangle_Pressed=False #button is not pressed - prepare to change sensitivity when button is pressed again

	def go_home(self):
		self.ser.write(b'home\r')
		time.sleep(180) # homing takes a few minutes ...
	
	def calculate_pos(self):
		#save the current robot position as P1 and ask it for the axis and joint coordinates of P1
		#self.ser.write(b'HERE AA \r')
		#read_and_wait(0.3)
		# self.ser.write(b'LISTPV P1 \r')
		read_and_wait(0.4)
		self.ser.write(b'LISTPV POSITION \r')

		robot_output = read_and_wait(1)
		output_after = robot_output.replace(': ', ':')
		pairs=(output_after.split())[2:-1] #separate in pairs of the form 'n:m'
		result_list=[]
		print(pairs)
		for pair in pairs:
			[key, value] = pair.split(':')
			result_list.append(int(value))
		self.joints = result_list[0:5]
		self.pos = result_list[5:10] 

		print('Calculate pos: ', self.pos)


	def get_pos(self):
		#function to get axis position values
		#run calculate_pos before running get_pos to update the values
		return self.pos

	def get_joints(self):
		#function to get joint values
		#run calculate_pos before running get_joints to update the joint values
		return self.joints
	
	def housekeeping(self):
		self.ser.close()
		print('housekeeping completed - exiting')	
		
	def move_joints(self,JointsDeltas, shared_queue ):
		 #for this function to work, 'JointsDelts' should come from an inverse kinematics function. 
        #this can be used instead of using move_axis. Move axis takes a position and move joints takes new joint values for each of the robot's movement
        #when a good inverse kinematics function is created, move_joints should give better results than move_axis
		#print('Old joints: ',self.joints)
		#print('Deltas joints: ', JointsDeltas)
		self.joints = [self.joints[i] + JointsDeltas[i] for i in range(len(JointsDeltas))]
		putInQueue = JointsDeltas + self.joints #this concatenates in a list with 10 values
		
		#to put joint values in queue - checking first if the values are not null
		deltasum=0
		for delta in JointsDeltas:
			deltasum+=pow(delta,2) #sum of squared values
		Threshold = 1
		if deltasum>Threshold:
			shared_queue.put(putInQueue)


def cameraRobot(Robot):
	#This is a child class of the class Robot. This means that it has all the methods and variables that the Robot class has, 
	#but with added methods specific to the camera robor
	def __init__(self, comPort, robotSpeed, sensitivityHigh, sensitivityLow):
		super(cameraRobot, self).__init__(self, comPort, robotSpeed, sensitivityHigh, sensitivityLow, 'camera') #inherite all methods from robot and do the init function of rthe robot class
		self.c= self.pos[2] #c=Z initial
		self.v=(c*((1/3) + (math.pi/6))) + self.pos[1] #v = equation + y

	def change_c(self, delta_c):
		self.c+= 10

	def get_c(self):
		return self.c
	
	def change_v(self, delta_v):
		self.v+=delta_v
		if self.v> self.c*(4/3+ math.pi/6):
			self.v = self.c*(4/3+ math.pi/6)
		elif self.v<0:
			self.v=0

	def get_v(self):
		return self.v
	
	def move_pos(self, new_pos, delta_pos , shared_queue):
		self.pos = new_pos
		putInQueue = delta_pos + self.pos #this concatenates in a list with 10 values
		
		#to put joint values in queue - checking first if the values are not null
		deltasum=0
		for delta in delta_pos:
			deltasum+=pow(delta,2) #sum of squared values
		Threshold = 1
		if deltasum>Threshold:
			shared_queue.put(putInQueue)
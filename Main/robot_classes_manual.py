import pygame
import time
import serial
import threading
import queue
import math
import numpy as np
import pickle

from joystick_functions import*
from foward_kinematics import*
from serial_functions import*


class Robot():
	def __init__(self, joystick=None,info_computer_share=None,atHome=False, comPort='COM4', speedHigh=16, speedLow=10):
		#initialization of the robot class
		#this function opens the serial connection between the robot and the computer, defined the robot's speed % and calibrates the robot
		if not atHome:
			self.ser = serial.Serial(comPort, baudrate=9600, bytesize=8, timeout=2, parity='N', xonxoff=0) #change for lab ------------------------HERE FOR LAB
			print("COM port in use: {0}".format(self.ser.name))		
		else:
			self.ser=False
		time.sleep(80)
		self.go_home()
		self.message= {'Q':0, '1':0,  'W':0,'2':0, 'E':0,'3':0,  'R':0,'4':0, 'T':0,'5':0,}

		self.initialize_pose_LinearRegressions()
		self.define_initial_variables(info_computer_share,speedLow,speedHigh,joystick)
		self.manual_mode ='J'
		self.go_to_initial_position()
		self.calculate_pos()

		self.update_bisturi_pos_shared()
		time.sleep(0.7)
		self.initial_manual_start(type=self.manual_mode)
		print('Bisturi Ready')

	def initialize_pose_LinearRegressions(self):
		with open("Joints_models.pickle", "rb") as f:
			while True:
				try:
					self.models_joints=pickle.load(f)
				except EOFError:
					break
		

		
	def go_to_initial_position(self):
		cartesian=['X','Y', 'Z', 'P', 'R']
		joints=['1','2', '3', '4', '5']
		initial_joints=[678,4492,-17459,-827,7823]#defined experimentally
		
		self.serial_write(b'DEFP A \r')
		time.sleep(0.4)
		self.serial_write(b'HERE A \r')
		time.sleep(0.4)
		for i, coordenate in  enumerate(initial_joints):	
			printToRobot=f'SETPV A {joints[i]} {int(coordenate)}\r'
			self.serial_write(printToRobot.encode('utf-8'))
			time.sleep(0.2)
		self.serial_write(b'MOVE A 600\r')
		time.sleep(6)


	def define_initial_variables(self, info_computer_share,speedLow,speedHigh,joystick  ):
		#for the function that shares information with other computer
		self.info_computer_share= info_computer_share 
			#List where self.sharedData[0] is the robot's calibration pos, self.sharedData[1] is the current position of the robot,
			#self.sharedData[2] is if joystick's square button state (to zero the coordenates shown on screen) 
		self.circle_Pressed = False 
		#Statements to use in the function update_speed
		self.speedLow = speedLow
		self.speed=self.speedHigh=speedHigh
		self.previousHighSpeed=True
		self.joystick = joystick
		self.x_Pressed=False
		self.tara_position =[0,0,0,0,0] #this is used to keep track of tara/zero 
		self.joints_last=[0,0,0,0,0]
		self.plan_to_cut_status =False
		self.triangle_Pressed = False 
		self.delta_cut=[0,0]
		self.count=0

	def get_serial(self):
		return self.ser
	
	def initial_manual_start(self,  type='J',*speed):
		if not speed: #if speed argument is not given, use normal speed of robot
			speed=self.speed
		
		self.serial_write(b'\r')
		self.serial_write(b'~ \r')
		time.sleep(0.5)
		self.serial_write(b's \r')
		time.sleep(0.5)
		self.serial_write(f'{speed} \r'.encode('utf-8'))
		if type=='J':
			self.info_computer_share['state'] = 0 #robot is running in joints mode
		else:
			self.info_computer_share['state'] = 1 #robot is running in xyz mode
			
		time.sleep(0.4)
		self.serial_write(f'{type} \r'.encode('utf-8'))
		time.sleep(0.2)

	def manual_start_midle(self, *speed):
		
		time.sleep(0.15)
		self.serial_write(b'\r')
		self.serial_write(b'~ \r')
		
		time.sleep(0.05)

	def manual_end(self):
		self.serial_write(b'\r')
		#clean_buffer(serial)
		time.sleep(0.05)
		self.serial_write(b'~\r')
		time.sleep(0.1)
		self.serial_write(b'\r')

	
	def get_speed(self):
		return self.speed
	
	def update_speed(self, button_Is_Pressed):
		if self.x_Pressed and button_Is_Pressed: #x was pressed and still is - no changes to sensitivity
			return False
		elif not self.x_Pressed and button_Is_Pressed: #x was not pressed and now is pressed - change sensitivity from Low to high or High to low
			self.joystick.rumble(0.1, 1, 300)
			if self.manual_mode == 'X': #robot was previously with low speed - and will now have high speed
				self.manual_end()
				time.sleep(0.1)
				self.calculate_pos()
				time.sleep(0.1)
				self.speed=self.speedHigh
				self.manual_mode = 'J'
				self.initial_manual_start(type = self.manual_mode)
				self.previousHighSpeed = True
				print('High speed')


			else: #robot was previously with high speed and will now have low speed
				self.manual_end()
				time.sleep(0.1)
				self.calculate_pos()
				time.sleep(0.1)
				self.speed=self.speedLow
				self.manual_mode = 'X'
				self.initial_manual_start(type = self.manual_mode)
				self.previousHighSpeed = False
				print('Low speed')

			time.sleep(0.3)
			self.serial_write(b'c \r')
			self.x_Pressed=True
			return True
		else: 
			self.x_Pressed=False #button is not pressed - prepare to change sensitivity when button is pressed again
			return False

	def tara(self, circleButton):
		if not circleButton: #circle is not pressed
			self.circle_Pressed=False 
			return

		if self.circle_Pressed and circleButton: #circe was pressed and still is - no changes
			return None
		elif not self.circle_Pressed and circleButton:  #circle was not pressed and now is pressed - send information of current pos
			self.manual_end()
			time.sleep(0.15)
			self.calculate_pos()
			self.joystick.rumble(0.4, 0.4, 200)
			self.manual_start_midle()
			self.tara_position=self.pos.copy()
			self.update_bisturi_pos_shared()
			self.circle_Pressed=True  

			#this is used to gather data
			""" if self.manual_mode=='X':
				delta_tara_pos=[-self.tara_position[i]+self.pos[i] for i in range(len(self.pos))]
				string=f'message:{self.message}  delta:{delta_tara_pos}   XYZ\n'
			else:
				
				delta_joints = [-self.joints_last[i]+self.joints[i] for i in range(len(self.joints))]
				string=f'message:{self.message}  delta:{delta_joints}   JOINTS\n'

			self.f.write(string) 
			self.joints_last=self.joints.copy()"""

		
	def evaluate_triangle(self, triangleButtons):
		if not triangleButtons: #triangle is not pressed
			self.triangle_Pressed=False
			return

		if self.triangle_Pressed and triangleButtons: #triangle was pressed and still is - no changes
			return None
		elif not self.triangle_Pressed and triangleButtons:  #triangle was not pressed and now is pressed - change plan_to_cut status
			self.triangle_Pressed=True
			if self.plan_to_cut_status:
				self.plan_to_cut_status =False
				self.delta_cut =[0,0]
				self.joystick.rumble(0.8, 0.8, 100)
				#robot is running normally
				if self.manual_mode == 'X':
					self.info_computer_share['state'] = 1 #xyz mode
				else:
					self.info_computer_share['state'] = 0  #joints mode	
				self.manual_start_midle()

			else:
				self.plan_to_cut_status =True
				self.joystick.rumble(0.8, 0.8, 400)
				self.info_computer_share['state'] = 2 #robot is preaparing for cut
				self.manual_end()
				time.sleep(0.5)
				self.calculate_pos() #while comunication of state with other computer, calculate position - which will be used when cutting


	def set_point(self,prefix, ptNumber, deltasXYZRP):
		cartesian=['X','Y', 'Z', 'P', 'R']
		self.serial_write(f'HERE {prefix}[{ptNumber}]\r'.encode('utf-8'))
		time.sleep(0.5)
		print('deltasxyz',deltasXYZRP)
		for i, delta in enumerate(deltasXYZRP):
			if delta!=0:
				self.serial_write(f'SETPVC {prefix}[{ptNumber}] {cartesian[i]} {int(self.pos[i]+delta)}\r'.encode('utf-8'))
				time.sleep(0.4)


	def set_path(self,prefix ):
		self.delta_cut[0]=abs(self.delta_cut[0])
		self.delta_cut[1]=-abs(self.delta_cut[1])
		
		#[500,-300]#x,z - some recomended values
		#POINT 1 - here
		deltasXYZRP=[0]*5
		self.set_point(prefix, 1, deltasXYZRP)
		time.sleep(0.4)
		#POINT 2 - tilt blade to 50 degrees
		deltasXYZRP[3]=-500-self.pos[3]
		self.set_point(prefix, 2, deltasXYZRP)
		time.sleep(0.4)	
		#POINT 3- down
		deltasXYZRP[2]=-abs(self.delta_cut[1])
		self.set_point(prefix, 3, deltasXYZRP)
		time.sleep(0.4)	
		#POINT 4 - tilt blade to 75 degrees
		deltasXYZRP[3]=-750-self.pos[3]
		self.set_point(prefix, 4, deltasXYZRP)
		time.sleep(0.4)				
		#POINT 5 - move in x 
		deltasXYZRP[0]=-abs(self.delta_cut[0])
		self.set_point(prefix, 5, deltasXYZRP)
		time.sleep(0.4)	
		#POINT 6 - move to orignal z position 
		deltasXYZRP[2]=0
		self.set_point(prefix, 6, deltasXYZRP)
		time.sleep(0.4)	
		#POINT 7 - move upwards, 5 cm away from gelatine
		deltasXYZRP[2]=500
		self.set_point(prefix, 7, deltasXYZRP)
		time.sleep(0.4)	


	def perform_cut(self):
		print('Preparing to cut')
		prefix='AA'
		nrpoints=7
		#creating an array of points
		self.serial_write(f"DIMP {prefix}[{20}]\r".encode('utf-8'))
		time.sleep(0.7)
		self.serial_write(f"DELP {prefix}\r".encode('utf-8'))
		time.sleep(0.2)
		self.serial_write(b"YES\r")
		time.sleep(0.2)
		self.serial_write(f"DIMP {prefix}[{nrpoints}]\r".encode('utf-8'))
		time.sleep(0.5)
		#defining the coordinates of the array of points:
		self.set_path(prefix)		
		time.sleep(1)
		#move through all positions
		self.serial_write(f'MOVES {prefix} 2 {nrpoints} {400 * nrpoints}\r'.encode('utf-8'))
		print('Cutting')
		time.sleep(4*nrpoints)
		print('Finished cutting')

	def plan_to_cut(self, axes, buttons):
		if buttons[15]: #touchpad clicked
			self.joystick.rumble(1, 1, 200)#rumble
			time.sleep(0.15)
			self.info_computer_share['state'] = 3 #robot is cutting
			self.joystick.rumble(0.3, 0.3, 10)#rumble while waiting for cut
			self.perform_cut()
			self.plan_to_cut_status =False
			self.delta_cut =[0,0]
			self.info_computer_share['cutting_plan']=[0,0]
			if self.manual_mode == 'X':
				self.info_computer_share['state'] = 1 #xyz mode
			else:
				self.info_computer_share['state'] = 0  #joints mode
			self.manual_start_midle()
			return
		sensitivity=2
		show=False
		if abs(axes[0])>0.4:#length of cut
			self.delta_cut[0]+=axes[0]*sensitivity
			show=True
		if abs(axes[3])>0.4: #depth
			self.delta_cut[1]+=axes[3]*sensitivity
			show=True		
		if show:
			self.info_computer_share['cutting_plan'] = [round(self.delta_cut[i]/10,1) for i in range(len(self.delta_cut))]

	def iterate(self,axes,buttons ):
		self.evaluate_triangle(buttons[3]) #enter or exit plan to cut mode if triangle is pressed
		if self.plan_to_cut_status:
			self.plan_to_cut(axes, buttons)
		else:
			self.count+=1
			self.tara(buttons[1]) #tara/zero the position shown on screen
			_ = self.update_speed(buttons[0]) #change speed if x is pressed
			self.manual_move(axes, buttons)
		

	def manual_move(self, axes,buttons ):
		if axes[1] < -0.4:
			self.serial_write(b'1 \r')
			self.message['1']+=1

		elif axes[1] > 0.4:
			self.serial_write(b'Q \r')
			self.message['Q']+=1

		elif axes[0] < -0.4:
			self.serial_write(b'2 \r')
			self.message['2']+=1

		elif axes[0] > 0.4:
			self.serial_write(b'W \r')
			self.message['W']+=1

		if axes[2] < -0.4:
			self.serial_write(b'4 \r')
			self.message['4']+=1

		elif axes[2] > 0.4:
			self.serial_write(b'R \r')
			self.message['R']+=1

		elif axes[3] < -0.4:
			self.serial_write(b'T \r')
			self.message['T']+=1

		elif axes[3] > 0.4:
			self.serial_write(b'5 \r')
			self.message['5']+=1
		
		if buttons[9] ==1:
			self.serial_write(b'3 \r')
			self.message['3']+=1
		
		elif buttons[10] ==1:
			self.serial_write(b'E \r') 
			self.message['E']+=1
		
				
		self.predict_next_pos()#update the self.pos info with an estimation, based on the input of manual movement
		self.update_bisturi_pos_shared()

	def predict_next_pos(self):
		deltas=[]
		deltas.append(self.message['Q']-self.message['1'])
		deltas.append(self.message['W']-self.message['2'])
		deltas.append(self.message['E']-self.message['3'])
		deltas.append(self.message['R']-self.message['4'])
		deltas.append(self.message['T']-self.message['5'])

		if self.manual_mode =='X':
			XYZ_sensitivity =[-4.725378787878788,
							-4.739024390243903,
							-4.646980255516841,
							-0.49864498644986455,
							3.0248419150858177] #from data
			

			for i in range(len(self.pos)):
				self.pos[i]=self.TruePose[i] + deltas[i]*XYZ_sensitivity[i]

		elif self.manual_mode =='J':
			
			#J_sensitivity = 35.79*self.speedHigh/self.speedLow #from data
			#pitch_sensitivity=69.275*self.speedHigh/self.speedLow #from data
			for i in range(len(self.joints)):
				model= self.models_joints[f'Joints_model{i}']
				delta = np.array([deltas[i]]).reshape(-1,1)
				change_joints=int(model.predict(delta))
				self.joints[i]=self.TrueJoints[i] + change_joints
				#self.pos=self.joints.copy()
			

			#self.pos =foward_kinematics(self.joints, 'bisturi') LEFT TO IMPLEMENT FULLY...

	def update_bisturi_pos_shared(self):
		self.info_computer_share['last_bisturi_pos']=[round(self.pos[i]-self.tara_position[i],0) for i in range(len(self.pos))]


	def go_home(self):
		self.serial_write(b'home\r')
		time.sleep(150) # homing takes a few minutes ...
	
	def calculate_pos(self):

		self.serial_write(b'LISTPV POSITION \r')
		time.sleep(0.05)
		if self.ser!=False:
			clean_buffer(self.ser)
			robot_output = read_and_wait(self.ser,0.15)
			print('ROBOT OUTPUT',robot_output )
			output_after = robot_output.replace(': ', ':').replace('>','')
			pairs=output_after.split() #separate in pairs of the form 'n:m'
			result_list=[]
			for pair in pairs:
				[key, value] = pair.split(':')
				result_list.append(int(value))
			self.joints = result_list[0:5]
			self.pos = result_list[5:10] 
		else:
			self.pos=[0,0,0,0,0]
			self.joints =[0,0,0,0,0]
		
		self.message= {'Q':0, '1':0,  'W':0,'2':0, 'E':0,'3':0,  'R':0,'4':0, 'T':0,'5':0}
		self.TruePose=self.pos.copy()
		self.TrueJoints=self.joints.copy()
		self.count=0

	def getcount(self):
		return self.count

	def get_last_pos(self):
		#function to get axis position values
		#run calculate_pos before running get_pos to update the values
		return self.pos

	def get_last_joints(self):
		#function to get joint values
		#run calculate_pos before running get_joints to update the joint values
		return self.joints
	
	def housekeeping(self):
		self.manual_end()
		time.sleep(0.5)
		if self.ser!=False: #if not atHome
			self.ser.close()
		print('housekeeping completed - exiting')	

	def serial_write(self, toWrite):
		if self.ser!=False: #if not atHome
			self.ser.write(toWrite)
		print('bisturi', toWrite)
		

class cameraRobot():
	#this robot's y axis should be parallel to the table/jellow
	def __init__(self, shared_camera_pos,atHome=False, robotSpeed=2, comPort='COM4' ):
		if not atHome:
			self.ser = serial.Serial(comPort, baudrate=9600, bytesize=8, timeout=2, parity='N', xonxoff=0) #change for lab ------------------------HERE FOR LAB
			print("COM port in use: {0}".format(self.ser.name))	
		else:
			self.ser=False	

		time.sleep(1)
		self.go_home()
		time.sleep(15)

		self.speed=robotSpeed
		self.shared_camera_pos = shared_camera_pos
		self.square_pressed=False 
		time.sleep(2)
		self.serial_write(b'\r')
		time.sleep(0.2)
		self.go_to_initial_position()
		self.calculate_pos()
		self.serial_write(f'SPEED {robotSpeed}\r'.encode('utf-8'))
		time.sleep(0.5)
		self.initial_manual_start()
		print('Camera ready')

	def go_to_initial_position(self):
		joints=['1','2', '3', '4', '5']
		initial_joints=[2021,-4748,-25945,14948,-15828] #defined experimentally	
		self.serial_write(b'DEFP A \r')
		time.sleep(0.4)
		self.serial_write(b'HERE A \r')
		time.sleep(0.4)
		for i, coordenate in  enumerate(initial_joints):	
			printToRobot=f'SETPV A {joints[i]} {int(coordenate)}\r'
			self.serial_write(printToRobot.encode('utf-8'))
			time.sleep(0.4)
		self.serial_write(b'MOVE A 600\r')
		time.sleep(6)

	def move(self, axes,buttons ):
		self.update_pos(buttons[2])
		if max(buttons[11:15]):
			if not self.arrowsPressed: #if some arrow button has just been pressed
				self.manual_end()
				time.sleep(0.2)
				self.calculate_pos()
				time.sleep(0.2)
				delta_z =0
				delta_x=0
				if buttons[11]-buttons[12]>0: #move up - arrow up pressed
					delta_z=500
				elif buttons[11]-buttons[12]<0: #move down - arrow down pressed
					delta_z=-500
				if buttons[13]-buttons[14]>0: #move backward - arrow left pressed
					delta_x=400
				elif buttons[13]-buttons[14]<0:#move foward - arrow right pressed
					delta_x=-400
				new_position = [self.pos[0]+delta_x, self.pos[1], self.pos[2]+delta_z, self.pos[3],self.pos[4] ]
				self.set_position(new_position)
				self.serial_write(b'Move A 200\r')
				time.sleep(2)
				self.pos = new_position
				self.shared_camera_pos.put(new_position) 
				self.arrowsPressed = True
				self.manual_start_midle()
		else:
			self.arrowsPressed = False
			self.manual_move(axes, buttons)

	def update_pos(self, square_button):
		if not square_button: #square is not pressed
			self.square_pressed=False 
			return
		if self.square_pressed and square_button: #square was pressed and still is - no changes
			return None
		elif not self.square_pressed and square_button:  #square was not pressed and now is pressed - get current position of robot
			self.manual_end()
			self.calculate_pos()
			time.sleep(0.2)
			self.manual_start_midle()
			self.square_pressed=True 
			time.sleep(0.3)
			self.serial_write(b'c \r')
			time.sleep(0.2)
		
	def manual_move(self,axes, buttons):
		message= {'Q':0, '1':0,  'R':0,'4':0}
		#pitch:
		if axes[4] > 0:
			self.serial_write(b'R \r')
			message['R']+=1
		elif axes[5] > 0:
			self.serial_write(b'4 \r')
			message['4']+=1
		#base joint
		if buttons[4] - buttons[6] <0:
			self.serial_write(b'1 \r')
			message['1']+=1
		elif buttons[4] - buttons[6] >0:
			self.serial_write(b'Q \r')
			message['Q']+=1
		
		for i in message:
			self.message[i]+=message[i]
		""" if max([self.message['1'],self.message['Q']]): #if there was movement on the base
			if not max([message['1'],message['Q']]): #and there was no movement of the base in this iteration
				self.calculate_pos() """
		self.predict_next_pos()
		self.shared_camera_pos.put(self.pos)


	def predict_next_pos(self, ):
		deltas=[]
		deltas.append(self.message['Q']-self.message['1'])
		deltas.append(self.message['R']-self.message['4'])
		base_sensitivity = 35.79 #from data
		pitch_sensitivity=69.275 #from data
		self.joints[0]= self.Truejoints[0] + deltas[0]*base_sensitivity#base
		self.joints[3]= self.Truejoints[3] + deltas[1]*pitch_sensitivity#pitch

		#self.pos =foward_kinematics(self.joints,'camera') LEFT TO IMPLEMENT FULLY
			

	def set_position(self, position_values):
		cartesian=['X','Y', 'Z', 'P', 'R']
		self.serial_write(b'HERE A\r')
		time.sleep(0.2)
		for i,coordenate in enumerate(position_values) :
			if self.pos[i] != position_values[i]:
				#printToRobot=f'SHIFT AA BY {i+1} {int(delta)} \r'
				printToRobot=f'SETPVC A {cartesian[i]} {int(coordenate)}\r'
				self.serial_write(printToRobot.encode('utf-8'))
				time.sleep(0.3)


	def initial_manual_start(self):
		self.serial_write(b'\r')
		self.serial_write(b'~ \r')
		self.serial_write(b's \r')
		self.serial_write(f'{self.speed} \r'.encode('utf-8'))
		time.sleep(0.3)
		self.serial_write(b'J \r')
		time.sleep(0.1)

	def manual_start_midle(self):
		self.serial_write(b'\r')
		self.serial_write(b'~ \r')
		self.serial_write(b's \r')
		self.serial_write(f'{self.speed} \r'.encode('utf-8'))
		#time.sleep(0.05)

	def manual_end(self):
		self.serial_write(b'\r')
		#clean_buffer(serial)
		self.serial_write(b'~\r')
		time.sleep(0.05)
		self.serial_write(b'\r')
	
	def go_home(self):
		self.serial_write(b'home\r')
		time.sleep(150) # homing takes a few minutes ...
	
	def calculate_pos(self):
		self.serial_write(b'\r')
		time.sleep(0.3)
		self.serial_write(b'LISTPV POSITION \r')
		time.sleep(0.05)
		if self.ser!=False: #not at home - in the lab
			clean_buffer(self.ser)
			robot_output = read_and_wait(self.ser,0.15)
			print('Robot output', robot_output)
			output_after = robot_output.replace(': ', ':').replace('>','')
			pairs=output_after.split() #separate in pairs of the form 'n:m'
			result_list=[]
			for pair in pairs:
				[key, value] = pair.split(':')
				result_list.append(int(value))
			self.joints = result_list[0:5]
			self.pos = result_list[5:10]
		else:
			self.pos=[1000,-1000,2034,234,1234]
			self.joints =[0,0,0,0,0]

		self.message= {'Q':0, '1':0,  'R':0,'4':0}
		self.TruePose=self.pos.copy()
		self.Truejoints=self.joints.copy()
		self.shared_camera_pos.put(self.pos)

	def get_pos(self):
		#function to get axis position values
		#run calculate_pos before running get_pos to update the values
		return self.pos

	def get_joints(self):
		#function to get joint values
		#run calculate_pos before running get_joints to update the joint values
		return self.joints
	
	def serial_write(self, toWrite):
		if self.ser!=False: #if not atHome
			self.ser.write(toWrite)
		print('camera',toWrite)
	
	def housekeeping(self):
		self.manual_end()
		time.sleep(0.5)
		if self.ser!=False: #not at home
			self.ser.close()    
		print('housekeeping completed - exiting')	
		
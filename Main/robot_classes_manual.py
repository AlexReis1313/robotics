import pygame
import time
import serial
import threading
import queue
import math

from joystick_functions import*

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

            except:
                pass
        else:
            deltat = time.time() - start_time
            if deltat>wait_time:
                flag = False
    return output

def clean_buffer(ser):
    flag = True
    while flag:
        if ser.in_waiting > 0:
            ser.readline()          
        else:
            break   


class Robot():
	def __init__(self, joystick,FPS,info_computer_share=None, comPort='COM3', speedHigh=16, speedLow=8):
		#initialization of the robot class
		#this function opens the serial connection between the robot and the computer, defined the robot's speed % and calibrates the robot
		
		self.ser = serial.Serial(comPort, baudrate=9600, bytesize=8, timeout=2, parity='N', xonxoff=0) #change for lab ------------------------HERE FOR LAB
		print("COM port in use: {0}".format(self.ser.name))		

		#for the function that shares information with other computer
		self.info_computer_share= info_computer_share 
			#List where self.sharedData[0] is the robot's calibration pos, self.sharedData[1] is the current position of the robot,
			#self.sharedData[2] is if joystick's square button state (to zero the coordenates shown on screen) 
		self.circle_Pressed = False 

		#Statements to use in the function update_speed
		self.speedLow = speedLow
		self.speed=self.speedHigh=speedHigh
		self.previousHighSpeed=False
		self.joystick = joystick
		self.triangle_Pressed=False
		self.tara_position =[0,0,0,0,0] #this is used to keep track of tara/zero 

		self.initial_manual_start()

		self.stop_program=False
		""" while True:
			calibration_result = self.calibrate(FPS)
			if  calibration_result: #calibration finished successfully
				self.calibratedPos = self.pos
				break
			elif calibration_result ==False:#keyboard interrupt
				self.stop_program=True
				break
			else: 
				self.manual_end()
				print('Try calibration again')  """

	def get_stop_program(self):
		return self.stop_program
	
	def get_serial(self):
		return self.ser

	def calibrate (self, FPS):	#function that is used to calibrate the robot in the begging of running the program
		CalibrationClock = pygame.time.Clock()
		print('Move the robot arm to the calibration position. Use the Joystick.\n', 'Click x when in calibration position')
		try:
			while True:
				# Handle events
				if pygame.event.peek(): #if there are events waiting in joystick queue
					axes, buttons, quit = get_joystick(self.joystick)
	
				if quit:
					return None #Calibration failed
				
				if buttons[0]: #x button pressed
					print('Calibration concluded with success')
					self.manual_end()
					self.calculate_pos()
					self.manual_start_midle()
					
					#self.sharedData[0]= self.calibratedPos = self.pos

					return True #Calibration ended
				
				self.manual_move(axes,buttons)
				CalibrationClock.tick(FPS)
		
		except KeyboardInterrupt:
			return False #Calibration Failed and exit program
		

	def initial_manual_start(self,  type='J',*speed):
		if not speed: #if speed argument is not given, use normal speed of robot
			speed=self.speed
		
		self.serial_write(b'\r')
		self.serial_write(b'~ \r')
		self.serial_write(b's \r')
		self.serial_write(f'{speed} \r'.encode('utf-8'))
		time.sleep(0.4)
		self.serial_write(f'{type} \r'.encode('utf-8'))
		time.sleep(0.2)

	def manual_start_midle(self, *speed):
		if not speed: #if speed argument is not given, use normal speed of robot
			speed=self.speed
		
		self.serial_write(b'\r')
		self.serial_write(b'~ \r')
		self.serial_write(b's \r')
		self.serial_write(f'{speed} \r'.encode('utf-8'))
		#time.sleep(0.05)

	def manual_end(self):
		self.serial_write(b'\r')
		#clean_buffer(serial)
		self.serial_write(b'~\r')
		time.sleep(0.05)
		self.serial_write(b'\r')


	def get_calibrationPos(self):#return calibration position
		#this function can be used when evaluating if the robot is going to colide with something. by returning the initial calibration position
		#for example, to check if a new position of the robot will make it colide with the table, we can check if the z axis of that position if some cm lower than the calibration positon z axis
		return self.calibratedPos
	
	def get_speed(self):
		return self.speed
	
	def update_speed(self, button_Is_Pressed):
		if self.triangle_Pressed and button_Is_Pressed: #triangle was pressed and still is - no changes to sensitivity
			return False
		elif not self.triangle_Pressed and button_Is_Pressed: #triangle was not pressed and now is pressed - change sensitivity from Low to high or High to low
			if not self.previousHighSpeed: #robot was previously with low speed - and will now have high speed
				self.manual_end()
				time.sleep(0.1)
				self.speed=self.speedHigh
				self.initial_manual_start(type = 'J')

				self.previousHighSpeed = True
				self.joystick.rumble(0, 0.8, 100)
				self.info_computer_share['Speed']='Speed High'
				print('High speed')


			else: #robot was previously with high speed and will now have low speed
				self.manual_end()
				time.sleep(0.1)
				self.speed=self.speedLow
				self.initial_manual_start(type = 'X')

				self.previousHighSpeed = False
				self.joystick.rumble(0.8, 0, 100)
				self.info_computer_share['Speed']='Speed Low'
				print('Low speed')

			self.triangle_Pressed=True
			return True
		else: 
			self.triangle_Pressed=False #button is not pressed - prepare to change sensitivity when button is pressed again
			return False

	def tara(self, circleButton):
		if not circleButton: #circle is not pressed
			self.circle_Pressed=False 
			return

		if self.circle_Pressed and circleButton: #circe was pressed and still is - no changes
			return None
		elif not self.circle_Pressed and circleButton:  #circle was not pressed and now is pressed - send information of current pos
			self.manual_end()
			self.calculate_pos()
			self.manual_start_midle()
			self.tara_position=self.pos
			self.update_bisturi_pos_shared()


		
		

	def manual_move(self, axes,buttons ):
		self.tara(buttons[1]) #tara/zero the position shown on screen
		_ = self.update_speed(buttons[3]) #change speed if triange is pressed
		if axes[1] < -0.2:
			self.serial_write(b'Q \r')

		elif axes[1] > 0.2:
			self.serial_write(b'1 \r')

		if axes[0] < -0.2:
			self.serial_write(b'2 \r')

		elif axes[0] > 0.2:
			self.serial_write(b'W \r')

		if axes[2] < -0.2:
			self.serial_write(b'4 \r')

		elif axes[2] > 0.2:
			self.serial_write(b'R \r')

		if axes[3] < -0.2:
			self.serial_write(b'T \r')

		elif axes[3] > 0.2:
			self.serial_write(b'5 \r')
		
		if buttons[9] ==1:
			self.serial_write(b'3 \r')

		elif buttons[10] ==1:
			self.serial_write(b'E \r') 
		
		#we should update the self.pos info with an estimation, based on the input of manual movement
		#self.update_bisturi_pos_shared()

	def update_bisturi_pos_shared(self):
		self.info_computer_share['last_bisturi_pos']=[self.pos[i]-self.tara_position[i] for i in range(len(self.pos))]


	def go_home(self):
		self.serial_write(b'home\r')
		time.sleep(180) # homing takes a few minutes ...
	
	def calculate_pos(self):
	
		self.serial_write(b'LISTPV POSITION \r')
		time.sleep(0.05)
		clean_buffer(self.ser)
		robot_output = read_and_wait(self.ser,0.15)
		output_after = robot_output.replace(': ', ':').replace('>','')
		pairs=output_after.split() #separate in pairs of the form 'n:m'
		result_list=[]
		for pair in pairs:
			[key, value] = pair.split(':')
			result_list.append(int(value))
		self.joints = result_list[0:5]
		self.pos = result_list[5:10] 


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
		self.ser.close()
		print('housekeeping completed - exiting')	

	def serial_write(self, toWrite):
		self.ser.write(toWrite)
		print('bisturi', toWrite)
		

class cameraRobot():
	#this robot's y axis should be parallel to the table/jellow
	def __init__(self, shared_camera_pos=None, robotSpeed=2, comPort='COM5' ):
		
		self.ser = serial.Serial(comPort, baudrate=9600, bytesize=8, timeout=2, parity='N', xonxoff=0) #change for lab ------------------------HERE FOR LAB
		print("COM port in use: {0}".format(self.ser.name))		
		self.speed=robotSpeed
		self.shared_camera_pos = shared_camera_pos

		self.calculate_pos()
		time.sleep(0.5)
		self.serial_write(b'DEFP A')
		time.sleep(0.8)
		self.serial_write(b'HERE A')
		time.sleep(0.5)
		
		self.serial_write(f'SPEED {robotSpeed}'.encode('utf-8'))
		time.sleep(0.5)
		self.initial_manual_start()
		print('Ready')

	

	def move(self, axes,buttons ):
		self.update_pos(buttons[6])
		if max(buttons[11:15]):
			if not self.arrowsPressed: #if some arrow button has just been pressed
				self.manual_end()
				if buttons[11]-buttons[12]>0: #move up - arrow up pressed
					delta_z=100
				elif buttons[11]-buttons[12]<0: #move down - arrow down pressed
					delta_z=-100

				if buttons[13]-buttons[14]>0: #move backward - arrow left pressed
					delta_x=100
				elif buttons[13]-buttons[14]<0:#move foward - arrow right pressed
					delta_x=-100

				new_position = [self.pos[0]+delta_x, self.pos[1], self.pos[2]+delta_z, self.pos[3],self.pos[4] ]

				self.set_position(new_position)
				self.serial_write(b'Move A \r')
				time.sleep(0.5)

				self.pos = new_position
				self.shared_camera_pos = new_position   #--- change for threads
				self.arrowsPressed = True
				self.manual_start_midle()

		else:
			self.arrowsPressed = False
			self.manual_pitch(axes)

	def update_pos(self, options_button):
		if not options_button: #circle is not pressed
			self.options_pressed=False 
			return

		if self.options_pressed and options_button: #circe was pressed and still is - no changes
			return None
		elif not self.options_pressed and options_button:  #circle was not pressed and now is pressed - send information of current pos
			self.manual_end()
			self.calculate_pos()
			self.manual_start_midle()


	def manual_pitch(self,axes):
		if axes[4] > 0:
			self.serial_write(b'R \r')

		elif axes[5] > 0:
			self.serial_write(b'4 \r')

	def set_position(self, position_values):
		
		cartesian=['X','Y', 'Z', 'P', 'R']

		for i,coordenate in enumerate(position_values) :
			if self.pos[i] != position_values[i]:
				#printToRobot=f'SHIFT AA BY {i+1} {int(delta)} \r'
				printToRobot=f'SETPVC A {cartesian[i]} {int(coordenate)}\r'
				print(printToRobot)
				self.serial_write(printToRobot.encode('utf-8'))
				time.sleep(0.5)


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
		time.sleep(180) # homing takes a few minutes ...
	
	def calculate_pos(self):
		time.sleep(0.3)
		self.serial_write(b'LISTPV POSITION \r')
		time.sleep(0.05)
		clean_buffer(self.ser)
		robot_output = read_and_wait(self.ser,0.15)
		output_after = robot_output.replace(': ', ':').replace('>','')
		pairs=output_after.split() #separate in pairs of the form 'n:m'
		result_list=[]
		for pair in pairs:
			[key, value] = pair.split(':')
			result_list.append(int(value))
		self.joints = result_list[0:5]
		self.pos = result_list[5:10] 


	def get_pos(self):
		#function to get axis position values
		#run calculate_pos before running get_pos to update the values
		return self.pos

	def get_joints(self):
		#function to get joint values
		#run calculate_pos before running get_joints to update the joint values
		return self.joints
	
	def serial_write(self, toWrite):
		self.ser.write(toWrite)
		print('camera',toWrite)
	

	def housekeeping(self):
		self.manual_end()
		time.sleep(0.5)
		self.ser.close()
		print('housekeeping completed - exiting')	
		
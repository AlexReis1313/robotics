import pygame
import time
import serial
import threading
import queue
import math


from joystick_functions import initialize_joystick, get_joystick

class Robot():
	def __init__(self, joystick,FPS,sharedData, comPort='COM4', speedHigh=30, speedLow=5):
		#initialization of the robot class
		#this function opens the serial connection between the robot and the computer, defined the robot's speed % and calibrates the robot
		
		self.ser = serial.Serial(comPort, baudrate=9600, bytesize=8, timeout=2, parity='N', xonxoff=0) #change for lab ------------------------HERE FOR LAB
		print("COM port in use: {0}".format(self.ser.name))		
		self.initial_manual_start()

		#for the function that shares information with other computer
		self.sharedData=sharedData 
			#List where self.sharedData[0] is the robot's calibration pos, self.sharedData[1] is the current position of the robot,
			#self.sharedData[2] is if joystick's square button state (to zero the coordenates shown on screen) 
		self.circle_Pressed = False 

		#Statements to use in the function update_speed
		self.speed=self.speedLow = speedLow
		self.speedHigh=speedHigh
		self.previousHighSpeed=False
		self.joystick = joystick
		self.triangle_Pressed=False

		while not self.calibrate(FPS): #this loop's condition runs the calibration function. If it returns True, the loop interior is never called
			self.manual_end()
			print('Try calibration again') 


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
					return False #Calibration failed
				
				if buttons[0]: #x button pressed
					print('Calibration concluded with success')
					self.manual_end()
					self.calculate_pos()
					self.manual_start_midle()
					
					self.sharedData[0]= self.calibratedPos = self.pos

					return True #Calibration ended
				
				self.manual_move(axes,buttons)
				CalibrationClock.tick(FPS)
		
		except KeyboardInterrupt:
			return False #Calibration Failed
		

	def initial_manual_start(self):
		self.ser.write(b'\r')
		self.ser.write(b'~ \r')
		self.ser.write(b's \r')
		self.ser.write(f'{self.speed} \r'.encode('utf-8'))
		self.ser.write(b'X \r')
		time.sleep(0.1)

	def manual_start_midle(self):
		self.ser.write(b'\r')
		self.ser.write(b'~ \r')
		self.ser.write(b's \r')
		self.ser.write(f'{self.speed} \r'.encode('utf-8'))
		time.sleep(0.05)

	def manual_end(self):
		self.ser.write(b'\r')
		#clean_buffer(serial)
		self.ser.write(b'~\r')
		time.sleep(0.05)

	def get_calibrationPos(self):#return calibration position
		#this function can be used when evaluating if the robot is going to colide with something. by returning the initial calibration position
		#for example, to check if a new position of the robot will make it colide with the table, we can check if the z axis of that position if some cm lower than the calibration positon z axis
		return self.calibratedPos
	
	def get_speed(self):
		return self.speed
	
	def update_speed(self, button_Is_Pressed):
		if self.triangle_Pressed and button_Is_Pressed: #triangle was pressed and still is - no changes to sensitivity
			return None
		elif not self.triangle_Pressed and button_Is_Pressed: #triangle was not pressed and now is pressed - change sensitivity from Low to high or High to low
			if not self.previousHighSpeed: #robot was previously with low speed - and will now have high speed
				self.ser.write(b's \r')
				time.sleep(0.1)
				self.ser.write(f'{self.speedHigh} \r'.encode('utf-8'))
				self.previousHighSpeed = True
				self.joystick.rumble(0, 0.8, 100)
				self.sharedData[0]='Speed High'
				print('High speed')

			else: #robot was previously with high speed and will now have low speed
				self.ser.write(b's \r')
				time.sleep(0.1)
				self.ser.write(f'{self.speedLow} \r'.encode('utf-8'))
				self.previousHighSpeed = False
				self.joystick.rumble(0.8, 0, 100)
				self.sharedData[0]='Speed Low'
				print('Low speed')
		else: 
			self.triangle_Pressed=False #button is not pressed - prepare to change sensitivity when button is pressed again

	def share_information(self, circleButton):
		if not circleButton: #circle is not pressed
			self.circle_Pressed=False 
			return

		if self.circle_Pressed and circleButton: #circe was pressed and still is - no changes
			return None
		elif not self.circle_Pressed and circleButton:  #circle was not pressed and now is pressed - send information of current pos
			self.manual_end()
			self.calculate_pos()
			self.manual_start_midle()
			self.sharedData[1]=self.pos

		
		 
			



	def manual_move(self, axes,buttons ):
		
		self.update_speed(buttons[3]) #change speed if triange is pressed
				
		if axes[1] < -0.2:
			self.ser.write(b'Q \r')

		elif axes[1] > 0.2:
			self.ser.write(b'1 \r')

		if axes[0] < -0.2:
			self.ser.write(b'2 \r')

		elif axes[0] > 0.2:
			self.ser.write(b'W \r')

		if axes[2] < -0.2:
			self.ser.write(b'4 \r')

		elif axes[2] > 0.2:
			self.ser.write(b'R \r')

		if axes[3] < -0.2:
			self.ser.write(b'T \r')

		elif axes[3] > 0.2:
			self.ser.write(b'5 \r')
		
		if buttons[9] ==1:
			self.ser.write(b'3 \r')

		elif buttons[10] ==1:
			self.ser.write(b'E \r') 

	def go_home(self):
		self.ser.write(b'home\r')
		time.sleep(180) # homing takes a few minutes ...
	
	def calculate_pos(self):
	
		clean_buffer(self.ser)
		self.ser.write(b'LISTPV POSITION \r')
		robot_output = read_and_wait(self.ser,0.2)
		output_after = robot_output.replace(': ', ':')
		pairs=(output_after.split())[2:-1] #separate in pairs of the form 'n:m'
		result_list=[]
		for pair in pairs:
			[key, value] = pair.split(':')
			result_list.append(int(value))
		self.joints = result_list[0:5]
		self.pos = result_list[5:10] 
		print('Calculate pos: ', self.pos)


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


def share_data_computers(buttons, robot, sharedData):
	robot.share_information(buttons[1])#if circle is pressed, send information to other computer 
	sharedData[2] = buttons[2] #if square is pressed, zero de coordenates shown on screen
	return None


def do_obstacle_avoidance(robot, sharedData):
	robot.calculate_pos()
	sharedData[1] = robot.get_last_pos()

	#this function should then do obstacle avoindace. This has not yet been done

	return None

def robot_controll_main_loop(sharedData):
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
			if quit:
				return
			if count> FPS: #happens one time each second
				count=0
				bisturi_robot.manual_end()
				do_obstacle_avoidance(bisturi_robot, sharedData)
				bisturi_robot.manual_start_midle()

			share_data_computers(buttons,bisturi_robot ,sharedData)

			count+=1
			bisturi_robot.manual_move(axes,buttons)
			clock.tick(FPS)
	
	except KeyboardInterrupt:
		pass
	finally:
		pygame.quit()
		for robot in robots:
			robot.housekeeping() #this ends the manual mode and closes the serial port

def computer_comunication_loop(sharedData):
	oldData= []*3
	clock = pygame.time.Clock()
	FPS=2
	serialOtherComputer = serial.Serial('COM3', baudrate=9600, bytesize=8, timeout=2, parity='N', xonxoff=0) 
	while True:
		oldData=sharedData
		SendString=''
		for data in oldData:
			SendString	+=f'{str(data)} \n'
		serialOtherComputer.write(f'{SendString}'.encode('uft-8'))
		clock.tick(FPS)


def main():
	sharedData = []*3
	
	robot_controll_thread = threading.Thread(target=robot_controll_main_loop, args=(sharedData))
	computer_comunication_thread = threading.Thread(target=computer_comunication_loop, args=(sharedData))
	
	robot_controll_thread.start()
	computer_comunication_thread.start()
	
	robot_controll_thread.join() #this ensures that the main function only stops when the joystick loop thread is done running. This function should only end after the housekeeping of the robots
	computer_comunication_thread.join()

if __name__ == "__main__":
    main()
	
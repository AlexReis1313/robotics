import pygame
import time
import serial

class Robot():
	def __init__(self, comPort, robotSpeed):
		#initialization of the robot class
		#this function opens the serial connection between the robot and the computer, defined the robot's speed % and calibrates the robot
		self.ser = serial.Serial(comPort, baudrate=9600, bytesize=8, timeout=2, parity='N', xonxoff=0)
		print("COM port in use: {0}".format(self.ser.name))
		self.ser.write(f'Speed {robotSpeed}\r'.encode('utf-8'))
		self.calibrate()

	def calibrate (self):	#function that is used to calibrate the robot in the begging of running the program
		print('Move the robot arm to the calibration position. Use the Teach Pendent')
		while True:
			if 'y'==input("Input 'y' when ready to calibrate (when the end effector is in the calibration position)"):
				self.ser.write(b'defp P1')
				self.read_and_wait(0)
				self.ser.write(b'HERE P1\r')
				self.read_and_wait(0)
				self.calibratedPos = self.get_pos()
				print('Calibration Finished')
				break
	
	def get_calibrationPos(self):#return calibration position
		#this function can be used when evaluating if the robot is going to colide with something. by returning the initial calibration position
		#for example, to check if a new position of the robot will make it colide with the table, we can check if the z axis of that position if some cm lower than the calibration positon z axis
		return self.calibratedPos

	def go_home(self):
		self.ser.write(b'home\r')
		time.sleep(180) # homing takes a few minutes ...
	
	def read_and_wait(self, wait_time): #this function reads the information that the robot outputs to the computer and returns it as a string
		serString = "" # Used to hold data coming over UART
		output = ""
		flag = True
		start_time = time.time()
		while flag:
			# Wait until there is data waiting in the serial buffer
			if self.ser.in_waiting > 0:
				# Read data out of the buffer until a carriage return / new line is found
				serString = self.ser.readline()
				# Print the contents of the serial data
				try:
					output = serString.decode("Ascii")
					print(serString.decode("Ascii"))
				except:
					pass
			else:
				deltat = time.time() - start_time
				if deltat>wait_time:
					flag = False
		return output

	def get_pos(self):
		self.pos=1 #function to get positions
		return self.pos

	def get_joints(self):
		self.joints=1 #function to get joint values
		return self.joints
	
	# def move_test1(self,buttons): #this function is not usefull for the final project, just to test
	# 	if buttons[0]==1:
	# 		print('going to point P1')
	# 		self.ser.write(b'MOVE P1\r')
	# 		time.sleep(0.5)
	# 		self.read_and_wait(2)

	# 	elif buttons[1]==1:
	# 		print('going to point P2')
	# 		self.ser.write(b'MOVE P2\r')
	# 		time.sleep(0.5)
	# 		self.read_and_wait(2)
	
	def move_axis(self, AxisDeltas, waitTime):
		axis_list=['x', 'y', 'z', 'P', 'R'] 	#auxiliary list to use in the for loop
		new_pos = self.pos + AxisDeltas			#self.pos should already be updated with the current position of the robot
												#new_pos is the position that we want the robot's end effector to travel to

		for i,pos in enumerate(new_pos) :
			printToRobot=f'setpv P1 {axis_list[i]} {pos} \r' #this prints to the robot strings in the form: 'setp P1 x 890'; for each of the 5 axis (890 is just an example, the real values are stored in 'new_pos')
			self.ser.write(printToRobot.encode('utf-8'))	#After this for loop, point P1 should have the coordenates that we want he robot to move to
			time.sleep(waitTime)

		self.ser.write(b'MOVE P1 \r')	#move to P1
		self.read_and_wait(waitTime)

	def move_joints(self, JointsDeltas, waitTime):
		#for this function to work, 'JointsDelts' should come from an inverse kinematics function. This is yet to be implemented
		for i,delta in enumerate(JointsDeltas) :
			printToRobot=f'setpv P1 {i+1} {delta} \r'
			self.ser.write(printToRobot.encode('utf-8'))
			time.sleep(waitTime)
		self.ser.write(b'MOVE P1 \r')
		self.read_and_wait(waitTime)
		
	def housekeeping(self):
		self.ser.close()
		print('housekeeping completed - exiting')	
	
	
	

def moverobots(axes, buttons, robots):
	#this function should: translate joystick movements to actual movements for the robots using inverse and foward kinematics and apply the movement to the robot's class function move(self)
	#eventually it can also check for colision
	print(axes)
	print(buttons)
	robots[0].move_test1(buttons)




def initialize_joystick():
	# Initialize Pygame
	pygame.init()
	# Initialize the gamepad
	pygame.joystick.init()
	# Check if any joystick/gamepad is connected
	if pygame.joystick.get_count() == 0:
		print("No gamepad found.")
		return
    # Initialize the first gamepad
	joystick = pygame.joystick.Joystick(0)
	joystick.init()
	print(f"Gamepad Name: {joystick.get_name()}")
	return joystick
	

def main(): 
	bisturi_robot=Robot('COM4')
	robots=[bisturi_robot] #eventually this list will have both the bisturi and camera robot
	# Counter 
	count = 0
	joystick = initialize_joystick()
	
	try:
		while True:
		# Handle events
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					return
				# Get gamepad input
				axes = [round(joystick.get_axis(i),3) for i in range(joystick.get_numaxes())]
				buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
				moverobots(axes, buttons, robots) #this function should: translate joystick movements to actual movements for the robots using inverse and foward kinematics and apply the movement to the robot's class function move(self)
				count += 1
	except KeyboardInterrupt:
		pass
	finally:
		# Clean up
		pygame.quit()
		for rob in robots:
			rob.housekeeping()


if __name__ == "__main__":
    main()

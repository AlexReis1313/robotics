import pygame
import time
import serial

class Robot():
	def __init__(self, comPort, robotSpeed, sensitivity):
		#initialization of the robot class
		#this function opens the serial connection between the robot and the computer, defined the robot's speed % and calibrates the robot
		self.ser = serial.Serial(comPort, baudrate=9600, bytesize=8, timeout=2, parity='N', xonxoff=0)
		print("COM port in use: {0}".format(self.ser.name))
		self.ser.write(f'Speed {robotSpeed}\r'.encode('utf-8'))
		self.calibrate()
		self.sensitivity = sensitivity

	def calibrate (self):	#function that is used to calibrate the robot in the begging of running the program
		print('Move the robot arm to the calibration position. Use the Teach Pendent')
		while True:
			if 'y'==input("Input 'y' when ready to calibrate (when the end effector is in the calibration position)"):
				self.ser.write(b'defp P1')
				self.read_and_wait(0)
				self.ser.write(b'TEACH P1\r')
				self.read_and_wait(0)
				self.calibratedPos = self.get_pos()
				print('Calibration Finished')
				break
	
	def get_calibrationPos(self):#return calibration position
		#this function can be used when evaluating if the robot is going to colide with something. by returning the initial calibration position
		#for example, to check if a new position of the robot will make it colide with the table, we can check if the z axis of that position if some cm lower than the calibration positon z axis
		return self.calibratedPos
	def get_sensitivity(self):
		return self.sensitivity

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

	def calculate_pos(self):
		#save the current robot position as P1 and ask it for the axis and joint coordinates of P1
		self.ser.write(b'TEACH P1')
		self.read_and_wait(0)
		self.ser.write(b'LISTPV P1')
		robot_output = self.read_and_wait(0)

		pairs=robot_output.split() #separate in pairs of the form 'n:m'
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
			printToRobot=f'SETPVC P1 {axis_list[i]} {pos} \r' #this prints to the robot strings in the form: 'setp P1 x 890'; for each of the 5 axis (890 is just an example, the real values are stored in 'new_pos')
			self.ser.write(printToRobot.encode('utf-8'))	 #After this for loop, point P1 should have the coordenates that we want he robot to move to
			time.sleep(waitTime)

		self.ser.write(b'MOVE P1 \r')	#move to P1
		self.read_and_wait(waitTime)

	def move_joints(self, JointsDeltas, waitTime):
		#for this function to work, 'JointsDelts' should come from an inverse kinematics function. This is yet to be implemented
		#this can be used instead of using move_axis. Move axis takes a position and move joints takes new joint values for each of the robot's movement
		#when a good inverse kinematics function is created, move_joints should give better results than move_axis
		for i,delta in enumerate(JointsDeltas) :
			printToRobot=f'setpv P1 {i+1} {delta} \r'
			self.ser.write(printToRobot.encode('utf-8'))
			self.read_and_wait(0)
		self.ser.write(b'MOVE P1 \r')
		self.read_and_wait(waitTime)
		
	def housekeeping(self):
		self.ser.close()
		print('housekeeping completed - exiting')	
	
	
def joystickToDeltas_bisturi(axes, buttons, robot):
	sensitivity = robot.get_sensitivity()
	x = axes[0]*sensitivity     #x axis controled by horizontal movemento of left analogue
	y = axes[1]*sensitivity     #y axis controled by vertical movemento of left analogue
	pitch = axes[2]*sensitivity # pitch axis controled by horizontal movement of right analogue
	roll = axes[3]*sensitivity  #roll axis controled by vertical movement of right analogue
	if buttons[4]== 1:			#L1 and L2 controll z axis
		z= sensitivity/2
	elif buttons[5]== 1:
		z= -sensitivity/2
	else:
		z=0
	return [x,y,z,pitch,roll]


def joystickToDeltas_camera(axes, buttons, robot):
	sensitivity = robot.get_sensitivity()
	#function not developed yet
	return None


def moverobots(axes, buttons, robots):
	#this function should: translate joystick movements to actual movements for the robots using inverse and foward kinematics and apply the movement to the robot's class function move(self)
	#eventually it can also check for colision
	waitTime = 0.1 #parameter to ajust - time that program waits for robot to move
	for i, robot in enumerate(robots):
		if i ==0:
			delta_Axis= joystickToDeltas_bisturi(axes, buttons,  robot)  #i=0 correspondos to robot 0 - robot with bisturi
		else:
			delta_Axis= joystickToDeltas_camera(axes, buttons,  robot)  #i=1 correspondos to robot 1 - robot with camera
		robot.calculate_pos
		robot.move_axis(delta_Axis, waitTime)


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
	bisturi_robot=Robot('COM4', 15, 50) #com port, robot speed and sensitivity for robot movement
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
		for robot in robots:
			robot.housekeeping()


if __name__ == "__main__":
    main()

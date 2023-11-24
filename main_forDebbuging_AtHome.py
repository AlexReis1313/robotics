import pygame
import time
import serial


"""This script should be the same as main.py but with all the serial.write to robot being replaced by print()
The joystick is also replaced by keyboard inputs

All original comands from main.py that were comment out should be inside a coment like this one
Other replacements are also done as needed"""




class Robot():
	def __init__(self, comPort, robotSpeed, sensitivityHigh, sensitivityLow):
		#initialization of the robot class
		#this function opens the serial connection between the robot and the computer, defined the robot's speed % and calibrates the robot
		""" 		self.ser = serial.Serial(comPort, baudrate=9600, bytesize=8, timeout=2, parity='N', xonxoff=0)
		print("COM port in use: {0}".format(self.ser.name))
		self.ser.write(f'Speed {robotSpeed}\r'.encode('utf-8')) """
		self.calibrate()
		self.pos = self.calibratedPos
		self.sensitivity = sensitivityHigh
		self.otherSensitivity= sensitivityLow
		self.L3Pressed=False

	def calibrate (self):	#function that is used to calibrate the robot in the begging of running the program
		print('Move the robot arm to the calibration position. Use the Teach Pendent')
		while True:
			if 'y'==input("Input 'y' when ready to calibrate (when the end effector is in the calibration position)"):
				"""self.read_and_wait(0)"""
				self.calculate_pos()
				self.calibratedPos = self.pos
				print('Calibration Finished')
				break
		 
	
	def get_calibrationPos(self):#return calibration position
		#this function can be used when evaluating if the robot is going to colide with something. by returning the initial calibration position
		#for example, to check if a new position of the robot will make it colide with the table, we can check if the z axis of that position if some cm lower than the calibration positon z axis
		return self.calibratedPos
	
	def get_sensitivity(self):
		return self.sensitivity
	
	def set_sensitivity(self, buttonPressed):
		if self.L3Pressed and buttonPressed: #L3 was pressed and still is - no changes to sensitivity
			return None
		elif not self.L3Pressed and buttonPressed: #L3 was not pressed and now is pressed - change sensitivity from Low to high or High to low
			self.sensitivity , self.otherSensitivity  = self.otherSensitivity,  self.sensitivity
		else:
			self.L3Pressed=False #button L3 is not pressed - prepare to change sensitivity when L3 is pressed again

	"""def go_home(self):
		self.ser.write(b'home\r')
		time.sleep(180) # homing takes a few minutes ..."""
	
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
					output = output + serString.decode("Ascii")
					#print('bef')
					print(serString.decode("Ascii"))
					#print('aft')
				except:
					pass
			else:
				deltat = time.time() - start_time
				if deltat>wait_time:
					flag = False
		return output

	def calculate_pos(self):
		#save the current robot position as P1 and ask it for the axis and joint coordinates of P1
		"""self.ser.write(b'HERE AA \r')
		self.read_and_wait(0.3)
		# self.ser.write(b'LISTPV P1 \r')
		self.read_and_wait(0.5)
		self.ser.write(b'LISTPV POSITION \r')

		robot_output = self.read_and_wait(1)
		
		print(robot_output)
		print('here')
		output_after = robot_output.replace(': ', ':')
		print(output_after)
		pairs=(output_after.split())[2:-1] #separate in pairs of the form 'n:m'
		result_list=[]
		print(pairs)
		for pair in pairs:
			[key, value] = pair.split(':')
			result_list.append(int(value))"""
		
		print('Calculating Position\n','HERE AA\n','LISTPV POSITION\n' ) #debugging replacement line

		self.joints = [100, 1000,1302, 2353, 2424]#debugging replacement line
		self.pos = [0,0,133,23,0]#debugging replacement line
	
	def get_pos(self):
		#function to get axis position values
		#run calculate_pos before running get_pos to update the values
		return self.pos

	def get_joints(self):
		#function to get joint values
		#run calculate_pos before running get_joints to update the joint values
		return self.joints

	def move_axis(self, AxisDeltas):
		axis_list=['x', 'y', 'z', 'P', 'R'] 	#auxiliary list to use in the for loop
		fixed_delta=[2,-2,0,0,0]
		
		self.calculate_pos()
		new_pos = [self.pos[i] + fixed_delta[i] for i in range(len(axis_list))]		#self.pos should already be updated with the current position of the robot
												#new_pos is the position that we want the robot's end effector to travel to
		
		print('New Pos: ',new_pos)
		print('Axis deltas: ',AxisDeltas)
		""" 		for i,pos in enumerate(new_pos) :
			printToRobot=f'SETPVC AA {axis_list[i]} {pos} \r' #this prints to the robot strings in the form: 'setp P1 x 890'; for each of the 5 axis (890 is just an example, the real values are stored in 'new_pos')
			self.ser.write(printToRobot.encode('utf-8'))	 #After this for loop, point P1 should have the coordenates that we want he robot to move to
			time.sleep(waitTime)

		self.ser.write(b'MOVE AA \r')	#move to P1
		self.read_and_wait(waitTime)
		 """
		for i,pos in enumerate(new_pos) :						#replacement line for debugging
			printToRobot=f'SETPVC AA {axis_list[i]} {pos} \n'	#replacement line for debugging
			print(printToRobot)									#replacement line for debugging

		print('MOVE AA \n')	#replacement line for debugging
		time.sleep(1)

	def move_joints(self, JointsDeltas):
		#for this function to work, 'JointsDelts' should come from an inverse kinematics function. This is yet to be implemented
		#this can be used instead of using move_axis. Move axis takes a position and move joints takes new joint values for each of the robot's movement
		#when a good inverse kinematics function is created, move_joints should give better results than move_axis
		self.calculate_pos()
		new_joints = [self.joints[i] + JointsDeltas[i] for i in range(len(JointsDeltas))]	
		
		print('Old joints: ',self.joints)
		print('New joints: ',new_joints)
		print('Deltas joints: ', JointsDeltas)

		"""for i,delta in enumerate(new_joints) :
			printToRobot=f'setpv AA {i+1} {int(delta)} \r'
			self.ser.write(printToRobot.encode('utf-8'))
			self.read_and_wait(0.2)
		self.ser.write(b'MOVE AA \r')
		self.read_and_wait(waitTime)"""

		for i,delta in enumerate(new_joints) :						#replacement line for debugging
			printToRobot=f'setpv AA {i+1} {int(delta)} \n'	#replacement line for debugging
			print(printToRobot)									#replacement line for debugging

		print('MOVE AA \n')	#replacement line for debugging
		time.sleep(1)

		
	def housekeeping(self):
		self.ser.close()
		print('housekeeping completed - exiting')	
	
	
def joystickToDeltas_bisturi(axes, buttons, robot):
	if buttons[11]==1:
		robot.set_sensitivity(True) #Change the sensitivity of the robot (between high and low sensitivity)
	else:
		robot.set_sensitivity(False) #inform robot that L3 button has been depressed
	
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
	print('joystickToDeltas_bisturi: \n')
	print('Axes:' , axes)
	print([x,y,z,pitch,roll])
	return [x,y,z,pitch,roll]


def joystickToDeltas_camera(axes, buttons, robot):
	sensitivity = robot.get_sensitivity()
	#function not developed yet
	return None


def moverobots(axes, buttons, robots):
	#this function should: translate joystick movements to actual movements for the robots using inverse and foward kinematics and apply the movement to the robot's class function move(self)
	#eventually it can also check for colision
	for i, robot in enumerate(robots):
		if i ==0:
			delta_Axis= joystickToDeltas_bisturi(axes, buttons,  robot)  #i=0 correspondos to robot 0 - robot with bisturi
		elif i==1:
			delta_Axis= joystickToDeltas_camera(axes, buttons,  robot)  #i=1 correspondos to robot 1 - robot with camera
		
		#robot.move_axis(delta_Axis, waitTime)
		robot.move_joints(delta_Axis)


def initialize_keyboard(): #altered from joystick to keyboard for debugging
	pygame.init()
	
	window = pygame.display.set_mode((300, 300))
	return window
	

def main(): 
	bisturi_robot=Robot('COM4', 15, 200, 1) #com port, robot speed and sensitivity values high and low for robot movement
	robots=[bisturi_robot] #eventually this list will have both the bisturi and camera robot
	# Counter 
	count = 0
	window = initialize_keyboard() #altered from joystick to keyboard for debugging
	
	try:
		while True:
		# Handle events
			events=pygame.event.get()
			try:
				event=events[-1] 
			#for event in pygame.event.get():
				if event.type == pygame.QUIT:
					return
				# Get gamepad input
				""" axes = [round(joystick.get_axis(i),3) for i in range(joystick.get_numaxes())]
				buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]	 """

				check=False
				axes=[0,0,0,0]
				buttons = [0,0,0,0,0,0,0,0,0,0,0,0] #12 buttons for debugging
				if event.type == pygame.KEYDOWN:
					if event.key == pygame.K_LEFT:
						axes[1] -=1
						check=True
					if event.key == pygame.K_RIGHT:
						axes[1] +=1
						check=True
					if event.key == pygame.K_UP:
						axes[2] -=1
						check=True
					if event.key == pygame.K_DOWN:
						axes[2] +=1
						check=True
					
				if check == True:
					print('Joystick values: \n')
					print(axes)
					print(buttons)
					moverobots(axes, buttons, robots) #this function should: translate joystick movements to actual movements for the robots using inverse and foward kinematics and apply the movement to the robot's class function move(self)
					count += 1
					#pygame.event.clear()

				window.fill(0)
			except:
				pass

				
	except KeyboardInterrupt:
		pass
	finally:
		# Clean up
		pygame.quit()
		"""for robot in robots:
			robot.housekeeping()"""


if __name__ == "__main__":
    main()

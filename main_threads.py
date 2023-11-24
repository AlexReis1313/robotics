
import pygame
import time
import serial
import threading
import queue


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
		self.L3Pressed=False
		
	def get_serial(self):
		return self.ser

	def calibrate (self):	#function that is used to calibrate the robot in the begging of running the program
		print('Move the robot arm to the calibration position. Use the Teach Pendent')
		while True:
			if 'y'==input("Input 'y' when ready to calibrate (when the end effector is in the calibration position)"):
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
		if self.L3Pressed and buttonPressed: #L3 was pressed and still is - no changes to sensitivity
			return None
		elif not self.L3Pressed and buttonPressed: #L3 was not pressed and now is pressed - change sensitivity from Low to high or High to low
			self.sensitivity , self.otherSensitivity  = self.otherSensitivity,  self.sensitivity
		else:
			self.L3Pressed=False #button L3 is not pressed - prepare to change sensitivity when L3 is pressed again

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
		print('Old joints: ',self.joints)
		print('Deltas joints: ', JointsDeltas)
		self.joints = [self.joints[i] + JointsDeltas[i] for i in range(len(JointsDeltas))]
		shared_queue.put(JointsDeltas)
		        
        
        
       
        
	
def joystickToDeltas_bisturi(axes, buttons, robot, shared_queue):
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

	robot.move_joints([x,y,z,pitch,roll], shared_queue)
	 


def joystickToDeltas_camera(axes, buttons, robot):
	sensitivity = robot.get_sensitivity()
	#function not developed yet
	return None


def moverobots(axes, buttons, robots, shared_queue):
	#this function should: translate joystick movements to actual movements for the robots using inverse and foward kinematics and apply the movement to the robot's class function move(self)
	#eventually it can also check for colision
	for i, robot in enumerate(robots):
		if i ==0:
			joystickToDeltas_bisturi(axes, buttons,  robot, shared_queue)  #i=0 correspondos to robot 0 - robot with bisturi
		elif i==1:
			joystickToDeltas_camera(axes, buttons,  robot)  #i=1 correspondos to robot 1 - robot with camera
			

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
	

def joystick_loop(shared_queue, robots):
	# Counter 
	count = 0
	joystickLoopClock = pygame.time.Clock()
	joystick = initialize_joystick()
	axes=[0,0,0,0]
	buttons = [0,0,0,0,0,0,0,0,0,0,0,0]
	FPS=10
	
	try:
		while True:
		# Handle events
			if axes!=[0,0,0,0] or buttons != [0,0,0,0,0,0,0,0,0,0,0,0]:
					moverobots(axes, buttons, robots, shared_queue) #this function should: translate joystick movements to actual movements for the robots using inverse and foward kinematics and update the shared_queue with new joint values
				
			events = pygame.event.get()
			for event in events:
				if event.type == pygame.QUIT:
					return
				# Get gamepad input
				axes = [round(joystick.get_axis(i),3) for i in range(joystick.get_numaxes())]
				buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]	
				for i, axe in enumerate(axes):
						if abs(axe)<0.2:
							axes[i]=0
				print('Joystick values: \n', axes, '\n', buttons)
				count += 1
			pygame.event.clear()
			joystickLoopClock.tick(FPS) # do not run loop faster than n times a second

	except KeyboardInterrupt:
		pass
	finally:
		# Clean up
		pygame.quit()
		for robot in robots:
			robot.housekeeping()













def set_joints(joint_delta, ser):
	
	for i,delta in enumerate(joint_delta) :
		if delta !=0:
			printToRobot=f'SHIFT AA BY {i+1} {int(delta)} \r'
			ser.write(printToRobot.encode('utf-8'))
			time.sleep(0.5)
	


def serial_comunication_loop(shared_queue, ser):
	count=0
	FPS=10
	clock_serial = pygame.time.Clock
	clock_serial.tick()
	while True:
		# Check if the shared queue has data
		if not shared_queue.empty() and count<3:
			jointDelta_values=[]
			while not shared_queue.empty():
				jointDelta_values.append(shared_queue.get())            # Retrieve joint values from the shared queue
			shared_queue.task_done()
			set_joints(jointDelta_values[-1], ser)
			needs_to_move=True
			count +=1
			

		elif needs_to_move:
			print('Moving')
			ser.write(b'MOVE AA \r')
			read_and_wait(ser,0.1)
			needs_to_move=False
			count=0

		clock_serial.tick(FPS) # do not run loop faster than n times a second

		
def main():
	bisturi_robot=Robot(15, 150, 1) #robot speed and sensitivity values high and low for robot movement
	robots=[bisturi_robot] #eventually this list will have both the bisturi and camera robot
	serBisturi=bisturi_robot.get_serial()
	shared_queue = queue.Queue() #this queue will save values for the robot's joint deltas


	bisturi_serial_thread = threading.Thread(target=serial_comunication_loop, args=(shared_queue, serBisturi))
	joystick_thread = threading.Thread(target=joystick_loop, args=(shared_queue,robots))
	
	joystick_thread.start()
	bisturi_serial_thread.start()
	
	joystick_thread.join() #this ensures that the main function only stops when the joystick loop thread is running. This function should only end after the housekeeping of the robots


if __name__ == "__main__":
    main()
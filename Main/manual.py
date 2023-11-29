import pygame
import time
import serial
import threading
import queue
import math

class Robot():
	def __init__(self, comPort, robotSpeed, sensitivityHigh, sensitivityLow, robot_type):
		#initialization of the robot class
		#this function opens the serial connection between the robot and the computer, defined the robot's speed % and calibrates the robot
		
		self.ser = serial.Serial(comPort, baudrate=9600, bytesize=8, timeout=2, parity='N', xonxoff=0) #change for lab ------------------------HERE FOR LAB
		print("COM port in use: {0}".format(self.ser.name))
		time.sleep(5)
		self.ser.write(f'Speed {robotSpeed}\r'.encode('utf-8'))
		self.calibrate(robot_type) 
		
		""" 
		self.ser=comPort # HERE FOR Home--------------------------------------------------
		self.calibratedPos=[0,0,0,0,0]
		self.joints=[0,0,0,0,0]
		 """
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
			return False, None
		elif not self.triangle_Pressed and buttonPressed: #triangle was not pressed and now is pressed - change sensitivity from Low to high or High to low
			self.sensitivity , self.otherSensitivity  = self.otherSensitivity,  self.sensitivity
			self.triangle_Pressed=True
			
			
			print('Sensitivity changed')
			if self.sensitivity >self.otherSensitivity:
				print('high')
				sensitivity = 'High'
			else:
				print('Low')
				sensitivity = 'Low'
			return True, sensitivity
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
		read_and_wait_original(self.ser, 0.4)
		self.ser.write(b'LISTPV POSITION \r')

		robot_output = read_and_wait_original(self.ser,1)
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

	def move_pos(self, PosDeltas,shared_queue ):#Try in LAB---
		self.pos = [self.pos[i] + PosDeltas[i] for i in range(len(PosDeltas))]
		putInQueue = PosDeltas + self.pos #this concatenates in a list with 10 values
		
		#to put pos values in queue - checking first if the values are not null
		deltasum=0
		for delta in PosDeltas:
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

def clean_buffer(ser):
    #this function reads the information that the robot outputs to the computer and returns it as a string
    #Used to hold data coming over UART
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

def manual_start(serial):
    serial.write(b'\r')
    clean_buffer(serial)
    time.sleep(0.5)
    serial.write(b'~ \r')
    time.sleep(0.1)
    serial.write(b's \r')
    time.sleep(0.1)
    serial.write(b'5 \r')
    time.sleep(0.1)
    print(read_and_wait_original(serial, 2))
    time.sleep(0.1)
    print('Manual start')

def manual_end(serial):
    serial.write(b'\r')
    clean_buffer(serial)
    time.sleep(1)
    serial.write(b'~\r')
    time.sleep(0.5)
    print('Manual end')
	
def manual_move(ser, axes,buttons):
	joystick=axes
	buttons=buttons

	if joystick[0] < -0.2:
		ser.write(b'1 \r')

	elif joystick[0] > 0.2:
		ser.write(b'Q \r')

	if joystick[1] < -0.2:
		ser.write(b'2 \r')

	elif joystick[1] > 0.2:
		ser.write(b'W \r')
          
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

def get_joystick(joystick):
	buttons=buttons_aux=[0]*16
	axes=[0]*6
	aux=0
	quit=False
	for event in pygame.event.get():
		aux+=1
		if event.type == pygame.QUIT:
			quit=True
				
		# Get gamepad input
		axes = [round(joystick.get_axis(i),3)+axes[i] for i in range(joystick.get_numaxes())]
		buttons_aux = [joystick.get_button(i) + buttons_aux[i] for i in range(joystick.get_numbuttons())]
	for i, button in enumerate(buttons_aux):
		if button>0:
			buttons[i]=1

	for i, axe in enumerate(axes):
		axe_true=axe/(max(1,aux))
		if abs(axe_true) > 0.2:
			axes[i]=round(axe_true,3)
		else:
			axes[i]=0
	return axes, buttons, quit


def main():
	joystick = initialize_joystick()
	FPS=40
	clock = pygame.time.Clock()
	bisturi_robot=Robot('COM4',5, -200, 5, 'bisturi')
	serBisturi=bisturi_robot.get_serial()
	robots=[bisturi_robot]
	manual_start(serBisturi)
	try:
		while True:
		# Handle events
			if pygame.event.peek(): #if there are events waiting in joystick queue
				axes, buttons, quit = get_joystick(joystick)
			if quit:
				return
			manual_move(serBisturi,axes,buttons)
			clock.tick(FPS)
	
	except KeyboardInterrupt:
		pass
	finally:
		manual_end(serBisturi)
		pygame.quit()
		for robot in robots:
			robot.housekeeping()


if __name__ == "__main__":
    main()
	
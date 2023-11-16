import pygame
import sys
import math
import time
import serial

class Robot():
	def __init__(self, comPort):
		
		self.ser = serial.Serial(comPort, baudrate=9600, bytesize=8, timeout=2, parity='N', xonxoff=0)
		print("COM port in use: {0}".format(self.ser.name))
		self.ser.write(b'Speed 15\r')

	def go_home(self):
		self.ser.write(b'home\r')
		time.sleep(180) # homing takes a few minutes ...
	
	def read_and_wait(self, wait_time):
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
	
	def move(self, axes, buttons):
		print('moving is not yet implemented')		#Here, get var, add axes and buttons to xyz coordinates, create pos, move to pos using serial port

	def get_pos(self):
		self.pos=1 #function to get positions
		return self.pos

	def get_joints(self):
		self.joints=1 #function to get joint values
		return self.joints
	
	def move_test1(self,buttons): #this function is not usefull for the final project, just to test
		if buttons[0]==1:
			print('going to point P1')
			self.ser.write(b'MOVE P1\r')
			time.sleep(0.5)
			self.read_and_wait(2)

		elif buttons[1]==1:
			print('going to point P2')
			self.ser.write(b'MOVE P2\r')
			time.sleep(0.5)
			self.read_and_wait(2)

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

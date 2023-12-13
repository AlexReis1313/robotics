import pygame
import time
import serial
import threading
import queue
import math


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
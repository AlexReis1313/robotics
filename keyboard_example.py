import pygame
import sys
import math

def moverobot(axes, buttons):
	print(axes)
	print(buttons)


def func():
	pygame.init()
	
	window = pygame.display.set_mode((300, 300))
	return window

def main():
	# Counter 
	count = 0
	
	# Initialize Pygame
	window = func()

	buttons = 0
	axes=0
	
	try:
		while True:
		# Handle events
			for event in pygame.event.get():
				if  event.type == pygame.QUIT:#event.key == pygame.K_ESCAPE or
					return
					
				# Get gamepad input
				#axes = [round(joystick.get_axis(i),3) for i in range(joystick.get_numaxes())]
				#buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
				check=False
				if event.type == pygame.KEYDOWN:
					if event.key == pygame.K_LEFT:
						axes -=1
						check=True
					if event.key == pygame.K_RIGHT:
						axes+=1
						check=True
					if event.key == pygame.K_UP:
						buttons+=1
						check=True
					if event.key == pygame.K_DOWN:
						buttons-=1
						check=True
				if check == True:
					moverobot(axes,buttons)

				window.fill(0)
				# Counting
				count += 1
				#print(count)
				# Process input
				#print(f"Axes: {axes}")
				#print(f"Buttons: {buttons}")
	
			
	finally:
		# Clean up
		pygame.quit()

if __name__ == "__main__":
    main()

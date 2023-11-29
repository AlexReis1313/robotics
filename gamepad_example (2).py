import pygame
import sys
import math
import os
os.environ['SDL_JOYSTICK_HIDAPI_PS4_RUMBLE'] = '1'

def get_joystick(joystick):
	buttons=buttons_aux=[0 for i in range(16)]
	axes=[0 for i in range(6)]
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

	denominator = max(1, aux)  
	for i, axe in enumerate(axes):
		axe_true=axe/denominator
		if abs(axe_true) > 0.2:
			axes[i]=round(axe_true,3)
		else:
			axes[i]=0

	return axes, buttons, quit


def main():
	# Counter 
	count = 0
	
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
	clock=pygame.time.Clock()
	
	try:
		while True:
		# Handle events
			axes, buttons, quit = get_joystick(joystick)

			""" events=pygame.event.get()
			event= events[-1] """
			""" buttons=buttons_aux=[0 for i in range(16)]
			axes=[0 for i in range(6)]
			aux=0
			for event in pygame.event.get():
				aux+=1
				if event.type == pygame.QUIT:
					return
				
				# Get gamepad input
				axes = [round(joystick.get_axis(i),3)+axes[i] for i in range(joystick.get_numaxes())]
				buttons_aux = [joystick.get_button(i) + buttons_aux[i] for i in range(joystick.get_numbuttons())]
			
			for i, button in enumerate(buttons_aux):
				if button>0:
					buttons[i]=1	
			axes =[round(axes[i]/aux,3) for i in range(len(axes))]	 """
			# Counting
			count += 1
			print(count)
			# Process input
			print(f"Axes: {axes}")
			print(f"Buttons: {buttons}")
			if buttons[2]==1:
				print(joystick.rumble(0, 0.6, 50))

			clock.tick(3)

	except KeyboardInterrupt:
		pass
		
	finally:
		# Clean up
		pygame.quit()

if __name__ == "__main__":
    main()

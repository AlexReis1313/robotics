import pygame
import time
import serial
import threading
import queue
import math
import socket
import cv2

from communication_server import *
from graph_interface import *

def receive_robot_data(info_computer_share):
    HEADER, FORMAT, DISCONNECT_MESSAGE, ADDR = define_constants()
    start_server(ADDR, HEADER, FORMAT, DISCONNECT_MESSAGE,info_computer_share )

def GUI_loop(info_computer_share):
    main_graphycs(info_computer_share) 



def main():
	info_computer_share = {'state': -1, 'last_bisturi_pos': [0,0,0,0,0],  'cutting_plan':[0,0], 'coliding':False }
							#state: -1 - Initializing
							#		0 - Running joints mode
							#		1 - Running xyz mode
							#		2 - Preparing for cut
							#		3 - Cutting
							#		4 - Finished running
	graphical_interface = threading.Thread(target=GUI_loop, args=(info_computer_share))
	receive_data_server = threading.Thread(target=receive_robot_data,args=(info_computer_share))
	
	graphical_interface.start()
	receive_data_server.start()
	
	graphical_interface.join() #this ensures that the main function only stops when the joystick loop thread is done running. This function should only end after the housekeeping of the robots
	receive_data_server.join()

if __name__ == "__main__":
    main()
	
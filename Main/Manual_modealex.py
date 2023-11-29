import pygame
import time
import serial
import threading
import queue
import math
import os
os.environ['SDL_JOYSTICK_HIDAPI_PS4_RUMBLE'] = '1' #this enables vibrations of PS4 controller


from serial_comunication import read_and_wait, wait_for_DONE, serial_comunication_loop, set_joints, clean_buffer, read_and_wait_original
from Robot_classes import Robot
from joystick_functions import initialize_joystick, get_joystick


def manual_start(serial):
    serial.write(b'\r')
    clean_buffer(serial)
    time.sleep(0.5)
    serial.write(b'~\r')
    time.sleep(0.1)
    serial.write(b's\r')
    time.sleep(0.1)
    serial.write(b'20\r')
    time.sleep(0.1)
    serial.write(b'X\r')
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



def manual_move_dias(ser):

	if joystick[0] < 0:
		ser.write(b'1 \r')

	elif joystick[0] > 0:
		ser.write(b'Q \r')

	if joystick[1] < 0:
		ser.write(b'2 \r')

	elif joystick[1] > 0:
		ser.write(b'W \r')




def manual_move(serial):
    start_time=time.time()
    FPS=60
    while time.time()- start_time<3:
        serial.write(b'1\r')
    print('Manual move')


def main():
    bisturi_robot=Robot('COM4',5, -200, 5, 'bisturi') #robot speed and sensitivity values high and low for robot movement
    serial=bisturi_robot.get_serial()
    prior_pos=bisturi_robot.get_pos()
    manual_start(serial)
    manual_move(serial)
    manual_end(serial)
    bisturi_robot.calculate_pos()
    posterior_pos=bisturi_robot.get_pos()
    print('Initial pos', prior_pos)
    print('Final pos', posterior_pos)
    print('Dif=', posterior_pos[0]-prior_pos[0] )




if __name__ == "__main__":
    main()
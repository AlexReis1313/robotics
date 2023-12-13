import pygame
import time
import serial
import threading
import queue
import math
import numpy as np
import random

from joystick_functions import *

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

            except:
                pass
        else:
            deltat = time.time() - start_time
            if deltat>wait_time:
                flag = False
    return output

def clean_buffer(ser):
    flag = True
    while flag:
        if ser.in_waiting > 0:
            ser.readline()          
        else:
            break   


class Robot():
    def __init__(self, comPort='COM3', speed=10):
        self.ser = serial.Serial(comPort, baudrate=9600, bytesize=8, timeout=2, parity='N', xonxoff=0) 
        print("COM port in use: {0}".format(self.ser.name))		
        self.speed = speed
        self.initial_manual_start()
    
    def initial_manual_start(self):
        self.ser.write(b'\r')
        self.ser.write(b'~ \r')
        self.ser.write(b's \r')
        self.ser.write(f'{self.speed} \r'.encode('utf-8'))
        time.sleep(0.4)
        self.ser.write(b'J \r')
        time.sleep(0.3)
    
    def manual_start_midle(self):
        self.ser.write(b'\r')
        self.ser.write(b'~ \r')
        self.ser.write(b's \r')
        self.ser.write(f'{self.speed} \r'.encode('utf-8'))
        time.sleep(0.5)
    
    def manual_end(self):
        self.ser.write(b'\r')
        #clean_buffer(serial)
        self.ser.write(b'~\r')
        time.sleep(0.2)
        self.ser.write(b'\r')
	
    def manual_move(self,axes, buttons, message ):
    
        if axes[1] < -0.2:
            self.ser.write(b'Q \r')
            message['Q']+=1

        elif axes[1] > 0.2:
            self.ser.write(b'1 \r')
            message['1']+=1

        if axes[0] < -0.2:
            self.ser.write(b'2 \r')
            message['2']+=1

        elif axes[0] > 0.2:
            self.ser.write(b'W \r')
            message['W']+=1

        if axes[2] < -0.2:
            self.ser.write(b'4 \r')

        elif axes[2] > 0.2:
            self.ser.write(b'R \r')
            message['R']+=1

        if axes[3] < -0.2:
            self.ser.write(b'T \r')
            message['T']+=1

        elif axes[3] > 0.2:
            self.ser.write(b'5 \r')
            message['5']+=1
        
        if buttons[9] ==1:
            self.ser.write(b'3 \r')
            message['3']+=1
        
        elif buttons[10] ==1:
            self.ser.write(b'E \r') 
            message['E']+=1

        return message
    
    def go_home(self):
        self.ser.write(b'home\r')
        time.sleep(180) # homing takes a few minutes ...
    
    def calculate_pos(self):
        time.sleep(0.4)
        self.ser.write(b'LISTPV POSITION \r')
        time.sleep(0.05)
        clean_buffer(self.ser)
        robot_output = read_and_wait(self.ser,0.15)
        output_after = robot_output.replace(': ', ':').replace('>','')
        pairs=output_after.split() #separate in pairs of the form 'n:m'
        result_list=[]
        for pair in pairs:
            [key, value] = pair.split(':')
            result_list.append(int(value))
        self.joints = result_list[0:5]
        self.pos = result_list[5:10] 
    
    def get_pos(self):
		#function to get axis position values
		#run calculate_pos before running get_pos to update the values
        print('Getting pos...')
        self.manual_end()
        time.sleep(1)
        self.calculate_pos()
        time.sleep(0.3)
        self.manual_start_midle()
        print('concluded')

        return self.joints
    def get_last_joints(self):
		#function to get joint values
		#run calculate_pos before running get_joints to update the joint values
        return self.joints

    def housekeeping(self):
        self.manual_end()
        time.sleep(0.5)
        self.ser.close()
        print('housekeeping completed - exiting')	
    

def robot_controll_main_loop():
    joystick = initialize_joystick()
    FPS=40
    clock = pygame.time.Clock()
    bisturi_robot=Robot()
    
    f=open('Data_joints.txt','w')
    message= {'Q':0, '1':0,  'W':0,'2':0, 'E':0,'3':0,  'R':0,'4':0, 'T':0,'5':0,}
    delta_pos=[]
    initial_pos = bisturi_robot.get_pos()
    robots=[bisturi_robot]
    count=0
    try:
        while True:
        
            # Handle events
            if pygame.event.peek():  # if there are events waiting in the joystick queue
                axes, buttons, quit = get_joystick(joystick)
                if count==-1:
                    count=0

            
            """  if count > FPS:  # happens one time each second
                print('saving')
                count = -1
                final_pos = bisturi_robot.get_pos()
                print(final_pos)
                for i in range(len(initial_pos)):
                    delta_pos.append(final_pos[i] - initial_pos[i])
                f.write(f'{delta_pos}   {message}\n')
                message = {'Q': 0, '1': 0, 'W': 0, '2': 0, 'E': 0, '3': 0, 'R': 0, '4': 0, 'T': 0, '5': 0}
                delta_pos = []
                initial_pos = final_p """

            tocheck=[abs(axes[i]) for i in range(len(axes)-2)]
            if max(buttons)>0 or max(tocheck)>0.2 and count!=-1:
                count += 1
                message = bisturi_robot.manual_move(axes, buttons, message)
                print('+count')
            clock.tick(FPS)

    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        f.close()

        for robot in robots:
            robot.housekeeping()  # this ends the manual mode and closes the serial port

if __name__ == "__main__":
    robot_controll_main_loop()
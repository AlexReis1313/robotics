import numpy as np
import math

def foward_kinematics(joints,robot_type): #left to implement
		if robot_type == 'bisturi':
			EF = 0.15+0.16
		elif robot_type == 'camera':
			EF = 0.18
		else:
			raise TypeError('Invalid robot_type') 
		theta1,theta2,theta3,theta4,theta5 = joints

		offset1 = 0 #I still need to see what are the robot values in homing position and subtract them here
		offset2 = 0
		offset3 = 0
		offset4 = 0 
		offset5 = 0

		theta1 += offset1
		theta2 += offset2
		theta3 += offset3
		theta4 += offset4
		theta5 += offset5

		T_01 = np.matrix([[math.cos(theta1),math.sin(theta1),0,0],
					[math.sin(theta1),math.cos(theta1),0,0],
					[0,0,1,0],
					[0,0,0,1]])
		T_12 = np.matrix([[1,0,0,0.05],
					[0,1,0,0],
					[0,1,0,3585],
					[0,0,0,1]])
		T_23 = np.matrix([[math.cos(theta2),-math.sin(theta2),0,0],
					[0,0,1,0],
					[-math.sin(theta2),-math.cos(theta2),0,0],
					[0,0,0,1]])
		T_34 = np.matrix([[0,1,0,0],
					[-1,0,0,0],
					[0,0,1,0.135]])
		T_45 = np.matrix([[math.cos(theta3),-math.sin(theta3),0,0.3],
					[math.sin(theta3),math.cos(theta3),0,0],
					[0,0,1,0],
					[0,0,0,1]])
		T_56 = np.matrix([[math.cos(theta4),-math.sin(theta4),0,0.35],
					[math.sin(theta4),math.cos(theta4),0,0],
					[0,0,1,0],
					[0,0,0,1]])
		T_67 = np.matrix([[0,-1,0,0],
					[1,0,0,0],
					[0,0,1,0],
					[0,0,0,1]])
		T_78 = np.matrix([[math.cos(theta5),-math.sin(theta5),0,0],
					[0,0,-1,0],
					[math.sin(theta5),math.cos(theta5),0,0],
					[0,0,0,1]])
		T_89 = np.matrix([[1,0,0,0],
					[0,1,0,0],
					[0,0,1,EF],
					[0,0,0,1]])
					
		T = T_01.dot(T_12).dot(T_23).dot(T_34).dot(T_45).dot(T_56).dot(T_67).dot(T_78).dot(T_89)
		pos = T[0:3,3]
		orientation = T[0:3,0:3] #If needed
		return pos.tolist() #return in list of values
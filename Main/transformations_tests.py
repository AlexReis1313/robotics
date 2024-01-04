import numpy as np

def do_obstacle_avoidance_es(bisturi_pose, camera_pose, L, info_computer_share):
	safety_distance = 200
	
	transformation_matrix = np.array([
		[-1, 0, 0, L],
		[0, -1, 0, 0],
		[0, 0, 1, 0],
		[0, 0, 0, 1]
	])

	# take the first 3 elements of the position vector, convert to mm and add a 1 for homogeneous form
	end_effector1_R1 = np.concatenate([np.array(bisturi_pose[:3]) / 10, np.array([1])])
	end_effector2_R2 = np.concatenate([np.array(camera_pose[:3]) / 10, np.array([1])])
	end_effector2_R1 = np.dot(transformation_matrix, end_effector2_R2)

	# Calculate the Euclidean distance between the two points
	distance = np.linalg.norm(end_effector1_R1 - end_effector2_R1)
	print('[SPANISH DISTANCE]',distance)
	# Check for collision and return result
	if distance > safety_distance:
		info_computer_share['colision']=False
		return False  # No collision
	else:
		info_computer_share['colision']=True
		return True  # Collision

def do_obstacle_avoidance(bisturi_pose, camera_pose, L, info_computer_share):
	safety_distance = 200
	
	transformation_matrix = np.array([
		[-1, 0, 0],
		[0, -1, 0],
		[0, 0, 1],
	])

	d = np.array([L,0,0]).reshape(3,1)
	camera_xyz = camera_pose[:3]
	X = transformation_matrix.dot(np.array(camera_xyz).reshape(3,1)) + d

	distance = np.linalg.norm(np.array(bisturi_pose[:3]).reshape(3,1) - X)
	print('[PORTUGUESE DISTANCE]', distance)
	
bisturi_pose = [99,99,14000,6,8]
camera_pose = [1,2,3,4,5]
L = 200

def main():
	
	info_computer_share = {'state': -1, 'last_bisturi_pos': [0,0,0,0,0],  'cutting_plan':[0,0],'coliding':False}

	test = do_obstacle_avoidance_es(bisturi_pose,camera_pose,L,info_computer_share)

	do_obstacle_avoidance(bisturi_pose, camera_pose, L, info_computer_share)

main()  
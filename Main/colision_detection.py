import numpy as np

def do_obstacle_avoidance(bisturi_pose, camera_pose, L, info_computer_share):
	safety_distance = 200
	


	transformation_matrix = np.array([
		[-1, 0, 0, L],
		[0, -1, 0, 0],
		[0, 0, 1, 0],
		[0, 0, 0, 1]
	])

	# take the first 3 elements of the position vector, convert to mm and add a 1 for homogeneous form
	end_effector1_R1 = np.concatenate([np.array(bisturi_pose[:3])/10 , np.array([1])])
	end_effector2_R2 = np.concatenate([np.array(camera_pose[:3]) /10, np.array([1])])
	end_effector2_R1 = np.dot(transformation_matrix, end_effector2_R2)
	
	# Calculate the Euclidean distance between the two points
	distance = np.linalg.norm(end_effector1_R1 - end_effector2_R1)
	
	# Check for collision and return result
	if distance > safety_distance:
		info_computer_share['coliding']=False
		return False  # No collision
	else:
		info_computer_share['coliding']=True
		return True  # Collision

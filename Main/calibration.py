import numpy as np
import cv2 as cv
import glob

# Termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Checkerboard size
checkerboard_size = (8, 6)

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)
objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real-world space
imgpoints = []  # 2d points in image plane.

# Get images from the calibration_folder
calibration_folder = 'calibration_folder'
images = glob.glob(calibration_folder + '/*.png')

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv.findChessboardCorners(gray, checkerboard_size, None)

    if not ret:
        print(fname)

    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(img, checkerboard_size, corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)

cv.destroyAllWindows()

print(f"Number of images: {len(images)}")
print(f"Number of valid calibration images: {len(objpoints)}")

# Perform camera calibration
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Save calibration parameters to an npz file
calibration_data = {
    'camera_matrix': mtx,
    'distortion_coefficients': dist,
    'rotation_vectors': rvecs,
    'translation_vectors': tvecs
}
np.savez('calibration_parameters.npz', **calibration_data)

# Print the calibration results
print("Camera matrix:\n", mtx)
print("\nDistortion coefficients:\n", dist)
print("\nTranslation vector:\n",tvecs)
print("\nRotation vectors:\n",rvecs)
print(np.shape(tvecs))
print(np.size(rvecs))

# Check the quality of the results
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error
print( "total error: {}".format(mean_error/len(objpoints)) )

# Load calibration parameters from the npz file
calibration_data = np.load('calibration_parameters.npz')
mtx = calibration_data['camera_matrix']
dist = calibration_data['distortion_coefficients']

# Checkerboard size
checkerboard_size = (8, 6)

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)
objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real-world space
imgpoints = []  # 2d points in image plane.

# Termination criteria for subpixel corner detection
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Load a new image for pose estimation
new_image_path = 'new_image.png'
new_img = cv.imread(new_image_path)

if new_img is None:
    print(f"Error: Could not read the new image at {new_image_path}")
    exit()

# Undistort the new image using calibration parameters
undistorted_img = cv.undistort(new_img, mtx, dist)

# Convert to grayscale
gray = cv.cvtColor(undistorted_img, cv.COLOR_BGR2GRAY)

# Find chessboard corners
ret, corners = cv.findChessboardCorners(gray, checkerboard_size, None)

if not ret:
    print("Chessboard corners not found in the new image.")
    exit()

# Refine corner positions
corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

# Draw the corners on the undistorted image
cv.drawChessboardCorners(undistorted_img, checkerboard_size, corners2, ret)

# Display the undistorted image with corners
cv.imshow('Undistorted Image with Corners', undistorted_img)
cv.waitKey(0)
cv.destroyAllWindows()

# Estimate pose using solvePnP
_, rvecs, tvecs, inliers = cv.solvePnPRansac(objp, corners2, mtx, dist)

# Print the estimated rotation and translation vectors
print("Estimated Rotation Vectors:\n", rvecs)
print("Estimated Translation Vectors:\n", tvecs)

# Create the transformation matrix
rotation_matrix, _ = cv.Rodrigues(rvecs)
transformation_matrix = np.hstack((rotation_matrix, tvecs))

# Print the transformation matrix
print("\nTransformation Matrix:\n", transformation_matrix)

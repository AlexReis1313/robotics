import cv2
import numpy as np
import time

def main_graphycs(info_computer_share):
    # Load the images you want to display
    image1 = cv2.imread('images/jointsmode.png', cv2.IMREAD_UNCHANGED)
    image2 = cv2.imread('images/XYZmanualmode.png', cv2.IMREAD_UNCHANGED)
    image3 = cv2.imread('images/cutplanning.png', cv2.IMREAD_UNCHANGED)
    camera_image = cv2.imread('images/camera.png', cv2.IMREAD_UNCHANGED)
    blank_image=np.ones((1000, 1000, 4), dtype = np.uint8)
    # List of images corresponding to states -1 until 4
    state_images = [image1, image2, image3, image3, blank_image]

    # Open a connection to the webcam (0 is usually the default camera), for the robot webcam, change to 1
    cap = cv2.VideoCapture(0)

    # Create a window for displaying both webcam stream and images
    cv2.namedWindow('Robot Camera', cv2.WINDOW_NORMAL)
    
    # Record the start time of the program
    start_time = time.time()
    while info_computer_share['state']!=4:
        # Read a frame from the webcam
        ret, frame = cap.read()

        # Get the height and width of the frame
        height, width, _ = frame.shape

        # Create a combined frame
        #combined_frame = np.zeros((height * 2, width, 3), dtype=np.uint8)
        #combined_frame = np.zeros((height, width, 3), dtype=np.uint8)

        # Copy the webcam stream to the top of the combined frame
        #combined_frame[:, :, :] = frame
        combined_frame=frame
        
        # Get information from the shared dictionary
        state = info_computer_share['state']
        last_bisturi_pos = info_computer_share['last_bisturi_pos'].copy()
        coliding = info_computer_share['coliding']

        
        # Display state information on the left upper corner of the video
        state_text = ""
        if state == -1:
            state_text = "Initializing"
        elif state == 0:
            state_text = "Running in joints mode"
        elif state == 1:
            state_text = "Running in XYZ mode"
        elif state == 2:
            state_text = "Preparing for cut"
            
            # Display the values of depth and length of cut when in state 2
            depth, length = info_computer_share['cutting_plan']
            depth_length_text = f"Depth of Cut: {depth} mm   Length of Cut: {length} mm"
            cv2.putText(combined_frame, depth_length_text, (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)     
            
        elif state == 3:
            state_text = "Cutting"
        elif state == 4:
            state_text = "Finished running"
        
        # Display the state information on the left upper corner of the video
        cv2.putText(combined_frame, state_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        # Display last bisturi position on the upper left corner of the video in white and slightly bigger
        last_bisturi_text = f"Bisturi Position (mm) -> (X,Y,Z)=({last_bisturi_pos[0]/10}, {last_bisturi_pos[1]/10}, {last_bisturi_pos[2]/10})"
        cv2.putText(combined_frame, last_bisturi_text, (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)  # Change size to 0.7 and color to white

        # Display "Imminent Collision" on the top right corner if coliding is True
        if coliding:
            cv2.putText(combined_frame, "IMMINENT COLLISION", (10, 95),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        """   
        # Convert the image to 3 channels (remove alpha channel) and concatenate to the bottom of the combined frame
        img_without_alpha = cv2.cvtColor(state_images[state], cv2.COLOR_BGRA2BGR)
        combined_frame[height:, :, :] = cv2.resize(img_without_alpha, (width, height))
        """



        # Resize the camera image to match the size of the other images
        camera_image_resized = cv2.resize(camera_image, (700, 350))  # Adjust the size as needed
        controls_image_resized = cv2.resize(state_images[state], (700, 350))
        black_bar = np.zeros((15,np.shape(controls_image_resized)[1] , np.shape(controls_image_resized)[2]), dtype = np.uint8)
        combined_frame_controls=np.vstack((np.vstack(( controls_image_resized,black_bar)),camera_image_resized ))


        # Display the camera image in a separate window
        cv2.imshow('Controls for Bisturi (Top) and for Camera (Bottom)', combined_frame_controls)

        # Calculate the time since the beginning of the program
        elapsed_time = time.time() - start_time
        time_text = f"Time: {elapsed_time:.2f} seconds"
        cv2.putText(combined_frame, time_text, (width -220, height - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

        # Display the combined frame
        cv2.imshow('Robot Camera', combined_frame)

        # Check for key press to exit the loop
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    # Release the video capture object and close the windows
    cap.release()
    cv2.destroyAllWindows()
    

if __name__ == '__main__':
    # Initialize the shared dictionary
    info_computer_share = {'state': -1, 'last_bisturi_pos': [0, 0, 0, 0, 0], 'cutting_plan': [0, 0], 'coliding': False}
    main_graphycs(info_computer_share)

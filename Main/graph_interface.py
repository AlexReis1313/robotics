import cv2
import numpy as np
import time

"Meter numa janela os valores" #Done
"Meter um if para se é joints ou xyz" #DONE
"Meter HOMING na janela preta" #DONE
"Trocar depth com length na janela" #DONE
"So contar tempo quando acaba o homing"
"Testar com Iphone" 
"Clean code"

def load_images():
    image1 = cv2.imread('images/jointsmode.png', cv2.IMREAD_UNCHANGED)
    image2 = cv2.imread('images/XYZmanualmode.png', cv2.IMREAD_UNCHANGED)
    image3 = cv2.imread('images/cutplanning.png', cv2.IMREAD_UNCHANGED)
    camera_image = cv2.imread('images/camera.png', cv2.IMREAD_UNCHANGED)
    homing_screen = cv2.imread('images/homing_screensaver.png', cv2.IMREAD_UNCHANGED)
    alpha_channel = np.ones((homing_screen.shape[0], homing_screen.shape[1], 1), dtype=np.uint8) * 255
    homing_screen = np.concatenate((homing_screen, alpha_channel), axis=2) #Add a color channel to match the other images

    # List of images corresponding to states -1 until 4
    state_images = [image1, image2, image3, image3, homing_screen], camera_image

    return state_images

def create_windows(aux_height,aux_width,camera_width, camera_height):

    cv2.namedWindow('Robot Camera', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Information window', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Controls for Bisturi (Top) and for Camera (Bottom)', cv2.WINDOW_NORMAL)
    
    cv2.moveWindow('Robot Camera', 0, 0)
    cv2.moveWindow('Information window', 0, camera_height + 30)
    cv2.moveWindow('Controls for Bisturi (Top) and for Camera (Bottom)', camera_width, 0)

    cv2.resizeWindow("Robot Camera", camera_width, camera_height)
    cv2.resizeWindow('Controls for Bisturi (Top) and for Camera (Bottom)', aux_width, aux_height*2)
    cv2.resizeWindow('Information window', camera_width, 150)

def main_graphics(info_computer_share):
    windows = False
    if windows:
        aux_height = 320
        aux_width = 500

        camera_width = 750
        camera_height = 450
        
    else:
        aux_height = 320
        aux_width = 605

        camera_width = 833
        camera_height = 450

    state_images, camera_image = load_images()
    create_windows(aux_height,aux_width,camera_width,camera_height)

    # Open a connection to the webcam (0 is usually the default camera), for the robot webcam, change to 1)
    cap = cv2.VideoCapture(0)

    # Record the start time of the program
    start_time = time.time()
    while info_computer_share['state']!=4:
        # Read a frame from the webcam
        ret, frame = cap.read()

        # Get the height and width of the frame
        height, width, _ = frame.shape
        
        # Get information from the shared dictionary
        state = info_computer_share['state']
        last_bisturi_pos = info_computer_share['last_bisturi_pos'].copy()
        coliding = info_computer_share['coliding']

        info_window = np.zeros((150,camera_width,3))
  
        # Display state information on the left upper corner of the video
        state_text = ""
        if state == -1:
            state_text = "Initializing"
        elif state == 0:
            state_text = "Running in joints mode"
            #last_bisturi_text = f"Joint's Values  -> (j1,j2,j3,j4,j5)=({last_bisturi_pos[0]}, {last_bisturi_pos[1]}, {last_bisturi_pos[2]}, {last_bisturi_pos[3]},{last_bisturi_pos[4]})"
            #cv2.putText(info_window, last_bisturi_text, (10, 60),
            #        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)  # Change size to 0.7 and color to white
        elif state == 1:
            state_text = "Running in XYZ mode"

        elif state == 2:
            state_text = "Preparing for cut"

            # Display the values of depth and length of cut when in state 2
            length, depth = info_computer_share['cutting_plan']
            depth_length_text = f"Depth of Cut: {depth} mm   Length of Cut: {length} mm"
            cv2.putText(info_window, depth_length_text, (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)     
            
        elif state == 3:
            state_text = "Cutting"

        elif state == 4:
            state_text = "Finished running"

        # Display last bisturi position on the upper left corner of the video in white and slightly bigger
        last_bisturi_text = f"Bisturi Position (mm) -> (X,Y,Z)=({last_bisturi_pos[0]/10}, {last_bisturi_pos[1]/10}, {last_bisturi_pos[2]/10})"
        cv2.putText(info_window, last_bisturi_text, (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)  # Change size to 0.7 and color to white
    
        # Display the state information on the left upper corner of the video
        cv2.putText(info_window, state_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        # Display "Imminent Collision" on the top right corner if coliding is True
        if coliding:
            cv2.putText(frame, "IMMINENT COLLISION", (10, 95),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        # Resize the camera image to match the size of the other images
        camera_image_resized = cv2.resize(camera_image, (aux_width, aux_height))  # Adjust the size as needed
        controls_image_resized = cv2.resize(state_images[state], (aux_width, aux_height))
        combined_frame_controls=np.vstack((controls_image_resized,camera_image_resized ))
        camera_resized = cv2.resize(frame, (camera_width,camera_height))

        # Calculate the time since the beginning of the program
        elapsed_time = time.time() - start_time
        time_text = f"Time: {elapsed_time:.2f} seconds"
        cv2.putText(info_window, time_text, (camera_width-240, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

        # Display the camera image in a separate window
        cv2.imshow('Controls for Bisturi (Top) and for Camera (Bottom)', combined_frame_controls)

        # Display the combined frame
        cv2.imshow('Robot Camera', camera_resized)
        cv2.imshow('Information window', info_window) 

        #print('width x height', width, height) #Iphone - 1920x1080 Mac - 1280x720
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
    main_graphics(info_computer_share)

import cv2
import numpy as np

# Load the images you want to overlay
image1 = cv2.imread('images/jointsmode.png', -1)
image2 = cv2.imread('images/XYZmanualmode.png', -1)
image3 = cv2.imread('images/cutmodeandcamera.png', -1)

# List of images
images = [image1, image2, image3]
current_image_index = 0

# Open a connection to the webcam (0 is usually the default camera)
cap = cv2.VideoCapture(0)
# tracker = cv2.TrackerCSRT_create()
tracker = cv2.legacy.TrackerMOSSE_create()

success, img = cap.read()
bbox = cv2.selectROI("Tracking", img, False)
tracker.init(img, bbox)

# Initial values for length_of_cut and angle_of_cut
length_of_cut = np.random.randint(50, 101)
angle_of_cut = np.random.randint(-360, 361)

# Initialize key variable
key = 0

while True:
    timer = cv2.getTickCount()
    success, img = cap.read()

    success, bbox = tracker.update(img)

    if success:
        # Draw bounding box
        x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
        cv2.rectangle(img, (x, y), ((x + w), (y + h)), (255, 0, 255), 3, 1)

        # Calculate center of the bounding box
        center_x = int(x + w / 2)
        center_y = int(y + h / 2)

        # Initialize key variable
        key = cv2.waitKey(1) & 0xFF

        # Check if 's' key is pressed to generate new random values
        if key == ord('s'):
            length_of_cut = np.random.randint(50, 101)
            angle_of_cut = np.random.randint(-360, 361)
            print('Length:', length_of_cut)
            print('Angle: ', angle_of_cut)

        # Check if 'n' key is pressed to switch between images
        elif key == ord('n'):
            current_image_index = (current_image_index + 1) % len(images)

        # Calculate endpoint of the line based on the angle
        endpoint = (int(center_x + length_of_cut * np.cos(np.radians(angle_of_cut))),
                    int(center_y - length_of_cut * np.sin(np.radians(angle_of_cut))))

        # Draw arrow starting from the center of the bounding box
        cv2.arrowedLine(img, (center_x, center_y), endpoint, (0, 255, 0), 2, tipLength=0.2)

        # Display tracking status
        cv2.putText(img, "Tracking", (75, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        # Display tracking lost
        cv2.putText(img, "Lost", (75, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Display frame rate
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
    cv2.putText(img, f"FPS: {int(fps)}", (75, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Overlay image on the video frame
    current_overlay_image = cv2.resize(images[current_image_index], (420, 270))
    overlay_height, overlay_width, _ = current_overlay_image.shape
    roi = img[0:overlay_height, img.shape[1] - overlay_width:img.shape[1]]
    mask = current_overlay_image[:, :, 3] / 255.0

    for c in range(0, 3):
        img[0:overlay_height, img.shape[1] - overlay_width:img.shape[1], c] = \
            img[0:overlay_height, img.shape[1] - overlay_width:img.shape[1], c] * (1 - mask) + \
            current_overlay_image[:, :, c] * mask

    # Show the tracking result
    cv2.imshow("Tracking", img)

    # Exit if 'q' is pressed
    if key == ord('q'):
        break

# Release the video capture object and close all windows
cap.release()
cv2.destroyAllWindows()

# Robotic Arm Surgical Cutting System

This project involves controlling two old-school robotic arms to perform a precise surgical cut on a gelatin-based flesh model. One robotic arm holds a scalpel, while the second controls the position of a camera, providing a video feed for the user (doctor) to perform the operation remotely, without being in the same room.

![Robotic Arms](https://github.com/user-attachments/assets/6f73f25c-0e7c-4509-9a66-85c609077619)


## Project Focus
The project primarily focuses on human interaction with the robotic system, emphasizing feedback to the user via:
- A **wireless PS4 controller** with haptic feedback (vibrations).
- **Information displayed on a custom GUI** to enhance user control and awareness.

## Results
The final system successfully performed the required task, allowing a human operator to control the robotic arms remotely with ease.

## Methods Used
- **Multithreading in Python** to enable parallel processing of robot control, GUI, PS4 controller interface, and robot interface.
- **Distributed computing** using socket communication between two computers:
  - One computer controlled the robotic arms on-site.
  - The second computer operated remotely, allowing the doctor to control the procedure.
- **USB serial communication** for interfacing with the robotic arms.
- **Calibration of robotic arms** to ensure precise movements.
- **Direct and inverse kinematics** for accurate motion planning.


![System Arquitecture](https://github.com/user-attachments/assets/6b71b52d-dec8-418d-a3d8-37ad0cffbb14)


## Full Report
For a detailed description, check out the full [Project Report](report.pdf).

## Media
<img width="1079" alt="XYZ_mode_GUI" src="https://github.com/user-attachments/assets/67547806-570c-499f-8f3b-b878f00b8ec0" />




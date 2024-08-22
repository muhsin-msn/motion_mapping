# # motion_mapping

# Project Title

This project involves teleoperating dual Panda robots in simulation. The left arm robot follows the Aruco marker pose . THe right arm robot follows the EE of UR10  robot.

## Requirements

- **ROS Noetic**
- **Aruco C++ Library** - https://sourceforge.net/projects/aruco/
- **Eigen Library**
- **CoppeliaSim** 
- **ROS USB Cam** - sudo apt install ros-noetic-usb-cam

## Instructions

Follow these steps to set up and run the project:

1. **Build the Workspace**
   - Navigate to workspace and build it using `catkin_make` or `catkin build`:
   

2. **Launch the Robots**
   - Launch the dual Panda robots by running:
     ```sh
     roslaunch robots_bringup panda_dual.launch
     ```
   - Launch the UR10 robot by running:
     ```sh
     roslaunch robots_bringup UR10_bringup.launch
     ```

3. **Launch the ROS USB Cam and Run Aruco Detection**
   - Launch the ROS USB camera and execute the Aruco marker detection:
     ```sh
     roslaunch usb_cam usb_cam-test.launch
      ```
     ```sh
     rosrun motion_mapping aruco.cpp
     ```

5. **Run the Right Panda Hand Script**
   - Execute the script to control the right Panda robot hand, which will follow the UR10 robot's end-effector:
     ```sh
     rosrun motion_mapping panda_right_hand.cpp
     ```

6. **Run the Left Panda Hand Script**
   - Execute the script to control the left Panda robot hand, which will follow the detected Aruco marker:
     ```sh
     rosrun motion_mapping panda_left_hand.cpp
     ```
7. **Launch the Simulation**
 - Open CoppeliaSim and load the simulation file and run the simulation:
   ```sh
   FrankaSim_bimanualmodified_final_UPDATED2.ttt
   ```
 

## Test Video

Check out a demonstration of the project in action: https://youtu.be/a3dL0kv0zsY

# MTRN4230_G21

## About

The following repository includes the program files for satisfaction of the MTRN4230 requirements. The objective of the program is to operate a Gazebo based ur5 robot arm simulation such that it correctly undertakes a pick-and-place task. This includes the user of computer based image processing techniques to identify objects based on shape and colour, and the correct actuation of the robotic arm using forward and inverse kinematics and trajectory planning.

### Prerequisites

All programs are to be run in the virtual environment provided by the course. The following is a list of necessary additional installations:
- OpenCV
- MoveIt
- Tkinter

### Break down of programs

#### 4230_G21_Object_Detection.py
The computer vision module of the program. Uses the data received from the ROS kinect sensor to identify different objects based on colour and shape. It returns the coordinates of the objects in the global reference frame for the pick-and-place operation.

#### 4230_G21_Kinematics.mat
The forward/inverse kinematic module of the program. Currently calculates the forward kinematics of the UR5e robot arm using the DH convention.

#### 4230_G21_GUI.py
Graphical user interface to operate the pick-and-place task. This allows the objects shape and colour to be defined, as well as the number of objects to be picked up. The primary library utilised for this operation is Tkinter.

#### 4230_G21_MotionPlanner.py and 4230_G21_TrajectoryPlanner.py
These files control the actuation of the robot arm such that it can achieve point-to-point control using the python library MoveIt. This is the base foundation for what will eventually become a trajectory plan. Significant contributions can be accredited to Huang Zhao (https://github.com/lihuang3/ur5_ROS-Gazebo).

#### URDF and SDF files
A variety of different URDF files are included that allow an array of differnet objects to be substantiated in the Gazebo simulation. These can be attributed to Huang Zhao (https://github.com/lihuang3/ur5_ROS-Gazebo) as the files were obtained from his work on the ur5_ROS_gazebo simulation and then edited to suit the application of this project. The Kinect SDF was also included as some minor modifications were also made to its content to allow it to statically spawn in Gazebo simulation.

## Built With

* [ROS](https://www.ros.org/) - Operating system used
* [Gazebo](http://gazebosim.org/) - Simulation environment used
* [python](https://www.python.org/) - Programming language used
* [OpenCV](https://opencv.org/) - Computer Vision library used
* [MoveIt](https://moveit.ros.org/) - Motion planning framework used
* [MATLAB](https://www.mathworks.com/products/matlab.html) - Desktop environment used

## Contributing

* [Huang Zhao](https://github.com/lihuang3/ur5_ROS-Gazebo) - Implementation of UR5 pick and place in ROS-Gazebo with a USB cam and vacuum grippers.

## Authors

* **Peter Bassett**
* **Edward Thomson**
* **Liam Pemberton**
* **Andrew Simpson**
* **Michael Irwin**
* **Yang Chen**
* **Matthew Lim**




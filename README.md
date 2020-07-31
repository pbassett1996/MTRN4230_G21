# MTRN4230_G21

## About

The following repository includes the program files for satisfaction of the MTRN4230 requirements. The objective of the program is to operate a Gazebo based ur5 robot arm simulation such that it correctly undertakes a pick-and-place task. This includes the user of computer based image processing techniques to identify objects based on shape and colour, and the correct actuation of the robotic arm using forward and inverse kinematics and trajectory planning.

### Prerequisites

All programs are to be run in the virtual environment provided by the course. The following is a list of necessary additional installations:
- OpenCV
- MoveIt
- Tkinter

### Setup
#### Computer Vision Setup
In one terminal type:
```bash
cd ~/simulation_ws/
catkin_make
source devel/setup.bash
export SVGA_VGPU10=0
roslaunch ur5_t2_4230 ur5_world.launch
```
After running the command make sure to orient the kinect facing the ground plane

In another terminal type:
```bash
cd ~/simulation_ws/src/mtrn4230_g21/src
python object_detection.py
```


### Break down of structure

- src/
    - Code associated with the pick and place solution.
    - trajectory/
        - Code for trajectory planning
- models/
    - Models of the kinect and items to be picked and placed.
- scripts/
    - Test and calculation scripts

### Break down of programs

#### Main_4230_G21.py
Main file for integration of Object Detection and Trajectory planner modules. Also now includes the GUI for user input.

#### ObjectDetection_4230_G21.py
The computer vision module of the program. Uses the data received from the ROS kinect sensor to identify different objects based on colour and shape. It returns the coordinates of the objects in the global reference frame for the pick-and-place operation.

#### Kinematics_4230_G21_.m
The forward/inverse kinematic module of the program. Currently calculates the forward kinematics of the UR5e robot arm using the DH convention.

#### 4230_G21_GUI.py
Graphical user interface to operate the pick-and-place task. This allows the objects shape and colour to be defined, as well as the number of objects to be picked up. The primary library utilised for this operation is Tkinter.

#### 4230_G21_MotionPlanner.py and TrajectoryPlanner_4230_G21_.py
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




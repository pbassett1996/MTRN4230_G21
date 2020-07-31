#!/usr/bin/env python
#Project: MTRN4230 Group Project
#Group: G21
#Date: 29/07/2020
#About: The following function coordinates the integration of the object detection and trajectory planner objects as well as the Graphical User Interface.

# General Libraries
import numpy as np
import math, random, time, os

# ROS
import rospy, sys, tf
from geometry_msgs.msg import *
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CameraInfo

#custom pick msg created for the project
from mtrn4230_t2_cv.msg import Pick

# OpenCV
import cv2, cv_bridge
from cv_bridge import CvBridgeError

# Robot Arm
from gazebo_msgs.srv import *

# Tkinter, GUI
import Tkinter as tk

# Project imports
from trajectory_planning.TrajectoryPlanner_4230_G21 import MoveItCartesianPath
from computer_vision.ObjectDetection_4230_G21 import ObjectDetection

models_folder = "../models"

colour = "all"
shape = "all"
kinect_height = 1.5 #meters above the ground
pos_array_x  = np.zeros(6)
pos_array_y = np.zeros(6)
object_num = 0
pick_objs = False

#Colour and shape identifier
def setVars(OD):
    global colour, shape, pick_objs, object_num
    colour = col.get()
    shape = shp.get()
    print "Picking up", str(colour), "objects of", str(shape), "shape/s."
    OD.UserInput(colour, shape)
    time.sleep(2)
    object_num = 0
    pick_objs = True

#Delete objects in Gazebo simulation
#Source: http://wiki.ros.org/rospy/Overview/Services
def delete_objects(number, delete, flag):
    if flag == True:
        item_name = "kinect1"
        d(item_name)
    for num in xrange(0, number+1):
        item_name = "Object_{0}".format(num)
        delete(item_name)
        time.sleep(0.1)
        
#Spawn objects in Gazebo simulation
#Source: http://wiki.ros.org/rospy/Overview/Services
def spawn_objects(number, spawn, obj1, obj2, obj3, obj4, orientation):
    print("Spawning objects into robot's field of view")
    global pos_array_x
    global pos_array_y
    for num in xrange(0,number+1):
        pos_x = random.uniform(0.2,0.7)
        pos_y = random.uniform(-0.5,0.5)
        pos_array_y[num] = pos_y
        pos_array_x[num] = pos_x
        item_name = "Object_{0}".format(num)
        item_pose = Pose(Point(x=pos_x, y = pos_y, z = 0), orientation)
        if(num >= 0 and num <= 1):
            spawn(item_name, obj1, "", item_pose, "world")
        elif(num == 2):
            spawn(item_name, obj3, "", item_pose, "world")
        elif(num > 2 and num <=4):
            spawn(item_name, obj3, "", item_pose, "world")
        elif(num >= 5):
            spawn(item_name, obj4, "", item_pose, "world")
        time.sleep(0.1)

#Close program
def close_windows(num, d):
    print("Closing down")
    delete_objects(num, d, False)
    cv2.destroyAllWindows()
    root.destroy()
    quit()

#Used to reset the objects in the kinects field of view
def reset_obj(num, s, d , obj1, obj2, obj3, obj4, orientation, OD):
    print("Resetting objects")
    global object_num, pick_objs
    OD.reset()
    pick_objs = False
    delete_objects(num, d, False)
    time.sleep(0.1)
    spawn_objects(num, s,obj1, obj2, obj3, obj4, orientation)
    object_num = 0

#Recrusive function that controls the motion planner using the object detection output   
def run(OD,pub, count, root, flag):
    global object_num, pick_objs
    count += 0.1
    
    #If the arm is not moving and we have pressed "go"
    #TODO: wait for robot to finish task before retasking
    if(pick_objs == True):
        pos = OD.get_coordinates() #Get object coordinates
        if(pos[object_num] != None):
            #send this to the pick place node
            #MP.pickup(pos[object_num][0], pos[object_num][1], 0.1)
            #
            msg = Pick()
            msg.x = pos[object_num][0]
            msg.y = pos[object_num][1]
            rospy.loginfo(msg)
            pub.publish(msg)

            object_num += 1

    if count < 100:
        root.after(100, run, OD,pub, count, root, flag)


if __name__ == "__main__":
    rospy.init_node('Main')
    pub = rospy.Publisher('pick_point_publisher', Pick,queue_size=100)

    #Block until service is available
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    rospy.wait_for_service("gazebo/get_model_state")

    #Quaternion orientation of kinect and objects
    orient_kinect = Quaternion(*tf.transformations.quaternion_from_euler(0,1.57,0))
    orient_obj = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))

    #Create objects for spawn and delete roservice
    s_kinect = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    s_obj = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
    d = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    states = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)


    #Open URDF and SDF model files
    with open(os.path.join(models_folder, "blue_box.urdf"), "r") as f:
        blue_box = f.read()
    with open(os.path.join(models_folder, "red_box.urdf"), "r") as f:
        red_box = f.read()
    with open(os.path.join(models_folder, "green_cube.urdf"), "r") as f:
        green_cube = f.read()
    with open(os.path.join(models_folder, "yellow_cylinder.urdf"), "r") as f:
        yellow_cylinder = f.read()
    with open(os.path.join(models_folder, "kinect.sdf"), "r") as f:
        kinect = f.read()

    #Spawn the kinect into the world
    model = states('kinect1', "link")
    if(model.success == False):
        item_pose = Pose(Point(x=0.5, y=0, z = kinect_height), orient_kinect)
        s_kinect("kinect1", kinect, "",item_pose, "world")
        time.sleep(1)
        
    #Spawn the objects for detection
    obj_num = 5
    spawn_objects(obj_num, s_obj, blue_box, red_box, green_cube, yellow_cylinder, orient_obj)
    
    try:
        OD = ObjectDetection(colour, shape)
    except KeyboardInterrupt:
        print "Shutting down."

    root = tk.Tk()
    root.title("MTRN4230 G21")
    root.geometry("500x300")
    root.configure(background='white smoke')

    col = tk.StringVar()
    col.set("all")
    shp = tk.StringVar()
    shp.set("all")

    title = tk.Text(root)
    title.insert(tk.INSERT,"MTRN4230 G21")

    titLable = tk.Label(root, text="MTRN4230 G21",bg='white smoke')
    titLable.place(x=150, y=10)
    titLable.config(font=("Space", 20))

    #colour drop down menu
    colorDrop = tk.OptionMenu(root, col,'red', 'blue', 'green', 'yellow', 'all')
    colorDrop.config(fg='white', bg='slate gray', width=10, borderwidth=5)
    colorDrop["menu"].config(fg='white', bg='slate gray')
    colorDrop.place(x=200,y=100)

    colLable = tk.Label(root, text="Colour")
    colLable.place(x=235, y=80)

    #shape drop down menu
    shapeDrop = tk.OptionMenu(root, shp,'Cube', 'Rect Box', 'Cylinder', 'all')
    shapeDrop.config(fg='white', bg='slate gray', width=10, borderwidth=5)
    shapeDrop["menu"].config(fg='white', bg='slate gray')
    shapeDrop.place(x=350,y=100)

    shapeLable = tk.Label(root, text="Shape")
    shapeLable.place(x=390, y=80)

    #start button
    startButton = tk.Button(root, text="START", command = lambda: setVars(OD), width=10, height=2, fg="white", bg="green4",borderwidth=5)
    startButton.place(x=50,y=75)
    #resetbutton
    resetButton = tk.Button(root, text="RESET", command = lambda: reset_obj(obj_num, s_obj, d, blue_box, red_box, green_cube, yellow_cylinder, orient_obj, OD), width=10,height=2, fg="white", bg="orange",borderwidth=5)
    resetButton.place(x=50,y=150)
    #stop button
    stopButton = tk.Button(root, text="STOP", command = lambda: close_windows(obj_num, d), width=10,height=2, fg="white", bg="red3",borderwidth=5)
    stopButton.place(x=50,y=225)
    
    # slider
    quantity = tk.Scale(root, from_=0, to=10, length=250, tickinterval=1, orient=tk.HORIZONTAL, width=20)
    quantity.place(x=200,y=200)

    time.sleep(0.1)
    count = 0
    flag = True
    run(OD,pub, count, root,flag)

    root.mainloop()
    #rospy.spin()

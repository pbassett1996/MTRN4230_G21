#!/usr/bin/env python
#Project: MTRN4230 Group Project
#Group: G21
#Date: 29/07/2020

import rospy, sys, tf
import moveit_commander
import message_filters
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from cv_bridge import CvBridgeError
from Motion_Planner import MoveItCartesianPath
from Object_Detection import ObjectDetection
import Tkinter as tk
import cv2, cv_bridge, random, time
import numpy as np
import math

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CameraInfo

colour = "all"
shape = "all"
kinect_height = 1.5 #meters above the ground
pos_array_x  = np.zeros(6)
pos_array_y = np.zeros(6)
object_num = 0
pick_objs = False

#Colour and shape identifier
def setVars(OD):
    global colour, shape, pick_objs
    colour = col.get()
    shape = shp.get()
    OD.UserInput(colour, shape)
    time.sleep(2)
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
    global pos_array_x
    global pos_array_y
    for num in xrange(0,number+1):
        pos_x = random.uniform(0.2,0.7)
        pos_y = random.uniform(-0.5,0.5)
        pos_array_y[num] = pos_y
        pos_array_x[num] = pos_x
        item_name = "Object_{0}".format(num)
        item_pose = Pose(Point(x=pos_x, y = pos_y, z = 0), orientation)
        if(num == 1):
            spawn(item_name, obj1, "", item_pose, "world")
        elif(num == 2):
            spawn(item_name, obj3, "", item_pose, "world")
        elif(num > 2 and num <=4):
            spawn(item_name, obj3, "", item_pose, "world")
        elif(num == 5):
            spawn(item_name, obj4, "", item_pose, "world")
        time.sleep(0.1)

#Close program
def close_windows(num, d):
    delete_objects(num, d, False)
    cv2.destroyAllWindows()
    root.destroy()
    quit()

#Used to reset the objects in the kinects field of view
def reset_obj(num, s, d , obj1, obj2, obj3, obj4, orientation, OD):
    global object_num, pick_objs
    OD.reset()
    pick_objs = False
    delete_objects(num, d, False)
    time.sleep(0.1)
    spawn_objects(num, s,obj1, obj2, obj3, obj4, orientation)
    object_num = 0

#Recrusive function that controls the motion planner using the object detection output   
def run(OD,MP, count, root, flag):
    global object_num, pick_objs
    count += 0.1
    
    #If the arm is not moving and we have pressed "go"
    if(MP.isMoving() == False and pick_objs == True):
        pos = OD.get_coordinates() #Get object coordinates
        if(pos[object_num] != None):
            MP.pickup(pos[object_num][0], pos[object_num][1], 0.1)
            object_num += 1

    if count < 100:
        root.after(100, run, OD, MP, count, root, flag)


if __name__ == "__main__":
    rospy.init_node('Main')

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
    with open("/home/javad/model_editor_models/unit_box/blue_box.urdf", "r") as f:
        blue_box = f.read()
    with open("/home/javad/model_editor_models/unit_box/red_box.urdf", "r") as f:
        red_box = f.read()
    with open("/home/javad/model_editor_models/unit_box/green_cube.urdf", "r") as f:
        green_cube = f.read()
    with open("/home/javad/model_editor_models/unit_box/yellow_cylinder.urdf", "r") as f:
        yellow_cylinder = f.read()
    with open("/home/javad/model_editor_models/kinect_ros/kinect.sdf", "r") as f:
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
        MP = MoveItCartesianPath()
    except KeyboardInterrupt:
        print "Shutting down."

    #Create tkinter window for GUI
    root = tk.Tk()
    root.title('MTRN4230 - Urm Group Asignment')
    root.geometry("250x100")

    #Create drop down list for colours
    col_lbl = tk.Label(root, text = "Enter colour input").place(relx = 0.1, rely = 0.1, anchor = 'nw')
    col = tk.StringVar()
    col.set("all")
    drop = tk.OptionMenu(root, col, "red", "blue", "green", "yellow", "all").place(relx = 0.8, rely = 0.05, anchor = 'ne')

    #Create drop down list for shapes
    shp_lbl = tk.Label(root, text="Enter shape input").place(relx = 0.1, rely = 0.45, anchor = 'nw')
    shp = tk.StringVar()
    shp.set("all")
    drop2 = tk.OptionMenu(root, shp, "Rect Box", "Cube", "Cylinder", "all").place(relx = 0.8, rely = 0.4, anchor = 'ne')

    #Create buttons
    goButton = tk.Button(root, text = "Go!", command = lambda: setVars(OD)).place(relx = 0.1, rely = 1.0, anchor = 'sw')
    deleteButton = tk.Button(root, text = "Exit", command = lambda: close_windows(obj_num, d)).place(relx = 0.9, rely = 1.0, anchor = 'se')
    resetButton = tk.Button(root, text = "Reset", command = lambda: reset_obj(obj_num, s_obj, d, blue_box, red_box, green_cube, yellow_cylinder, orient_obj, OD))
    resetButton.place(relx = 0.5, rely = 1.0, anchor = 's')

    time.sleep(0.1)
    count = 0
    flag = True
    run(OD,MP, count, root,flag)

    root.mainloop()
    #rospy.spin()

#!/usr/bin/env python
#BEGIN ALL
#Project: Object Detection Module for MTRN4230 Group Project
#Group: G21
#Date: 15/07/2020

import rospy, tf
from sensor_msgs.msg import Image
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
import Tkinter as tk
import cv2, cv_bridge, random, time
import numpy as np


colour = "all"
shape = "all"

class ObjectDetection:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/color/image_raw', 
                                  Image, self.image_callback)
    #Shape Detection
    def ColourDetection(self):
        if(colour == "all"):
            hue_min = 0
            hue_max = 255
        elif(colour == "green"):
            hue_min = 33
            hue_max = 65
        elif(colour == "red"):
            hue_min = 0
            hue_max = 13
        elif(colour == "blue"):
            hue_min = 65
            hue_max = 255
        elif(colour == "yellow"):
            hue_min = 5
            hue_max = 59
        else:
            hue_min = 0
            hue_max = 255

        return hue_min, hue_max

    def ShapeDetection(self,  img):
        imgCopy = img.copy()
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hue_min, hue_max = self.ColourDetection()

        lower = np.array([hue_min, 1, 220])
        upper = np.array([hue_max, 255, 255])
        mask = cv2.inRange(imgHSV, lower, upper)
        imgResult = cv2.bitwise_and(img, img, mask=mask)

        imgGray = cv2.cvtColor(imgResult,  cv2.COLOR_BGR2GRAY)
        imgDil = mask.copy()

        contours,  hierarchy = cv2.findContours(imgDil,  cv2.RETR_EXTERNAL,  cv2.CHAIN_APPROX_NONE)[-2:]
        for cnt in contours:
            cv2.drawContours(imgDil,  cnt,  -1,  (255, 0, 0), 3)
            peri = cv2.arcLength(cnt,  True)
            approx = cv2.approxPolyDP(cnt,  0.02*peri,  True)
            objCor = len(approx)
            x,  y,  w,  h = cv2.boundingRect(approx)
            objectType = ""

            if objCor == 3: objectType = "Triangle"
            elif objCor == 4:
                aspRatio = w/float(h)
                if aspRatio > 0.9 and aspRatio < 1.1: objectType = "Cube"
                else: objectType = "Rect Box"
            elif objCor >= 8:
                objectType = "Cylinder"
            cv2.rectangle(img,  (x, y),  (x+w,  y+h),  (0,  255,  0), 2)
            if(shape == objectType or shape == "all"):
                cv2.putText(imgCopy,  objectType,  (x+(w/2)+10,  y+(h/2)-10),  cv2.FONT_HERSHEY_COMPLEX,  0.3,  (255, 255, 255),  1)
            
            cv2.namedWindow("window")
            cv2.imshow("window", imgCopy)
            cv2.waitKey(5)
        
    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        self.ShapeDetection(image)

def setVars():
    global colour
    global shape 
    colour = col.get()
    shape = shp.get()


def delete_objects(number, delete):
    item_name = "kinect1"
    d(item_name)
    for num in xrange(0, number):
        item_name = "Object_{0}".format(num)
        delete(item_name)

def spawn_objects(number, spawn, obj1, obj2, obj3, obj4, orientation):
    for num in xrange(0,number):
        pos_x = random.uniform(-0.75,0.75)
        pos_y = random.uniform(-0.75,0.75)
        item_name = "Object_{0}".format(num)
        item_pose = Pose(Point(x=pos_x, y = pos_y, z = 0), orientation)
        if(num == 1):
            spawn(item_name, obj1, "", item_pose, "world")
        elif(num == 2):
            spawn(item_name, obj2, "", item_pose, "world")
        elif(num > 2 and num <=4):
            spawn(item_name, obj3, "", item_pose, "world")
        elif(num > 4 and num <=6):
            spawn(item_name, obj4, "", item_pose, "world")

def close_windows(num, s):
    delete_objects(num, s)
    cv2.destroyAllWindows()
    root.destroy()
    quit()

#Initialisation
rospy.init_node('objectDetection')
rospy.wait_for_service("gazebo/delete_model")
rospy.wait_for_service("gazebo/spawn_sdf_model")
rospy.wait_for_service("gazebo/spawn_urdf_model")

#Quaternion orientation of kinect and objects
orient_kinect = Quaternion(*tf.transformations.quaternion_from_euler(0,1.57,0))
orient_obj = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))

#Create objects for spawn and delete roservice
s_kinect = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
s_obj = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
d = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

#Open URDF and SDF model files
with open("/home/javad/model_editor_models/unit_box/blue_box.urdf", "r") as f:
    blue_box = f.read()
with open("/home/javad/model_editor_models/unit_box/red_box.urdf", "r") as f:
    red_box = f.read()
with open("/home/javad/model_editor_models/unit_box/green_cube.urdf", "r") as f:
    green_cube = f.read()
with open("/home/javad/model_editor_models/unit_box/yellow_cylinder.urdf", "r") as f:
    yellow_cylinder = f.read()
with open("/home/javad/model_editor_models/kinect_ros/model.sdf", "r") as f:
    kinect = f.read()

#Spawn the kinect into the world
item_pose = Pose(Point(x=0, y=0, z = 1.5), orient_kinect)
s_kinect("kinect1", kinect, "",item_pose, "world")

#Spawn the objects for detection
spawn_objects(7, s_obj, blue_box, red_box, green_cube, yellow_cylinder, orient_obj)

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
goButton = tk.Button(root, text = "Go!", command = setVars).place(relx = 0.3, rely = 1.0, anchor = 'sw')
deleteButton = tk.Button(root, text = "Exit", command = lambda: close_windows(7, d)).place(relx = 0.7, rely = 1.0, anchor = 'se')

#Begin detecting objects
follower = ObjectDetection()

root.mainloop()
rospy.spin()



# END ALL

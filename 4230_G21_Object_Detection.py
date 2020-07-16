#!/usr/bin/env python
#BEGIN ALL
#Project: Object Detection Module for MTRN4230 Group Project
#Group: G21
#Date: 15/07/2020

import rospy, tf
import message_filters
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CameraInfo
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from cv_bridge import CvBridgeError
import Tkinter as tk
import cv2, cv_bridge, random, time
import numpy as np
import math


colour = "all"
shape = "all"
kinect_height = 1.5 #meters above the ground

class ObjectDetection:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge() 
        self.img = None
        self.depth_array = None
        self.point_cloud = None
        self.R_matrix = None
        #Create subscribers to depth sensor messages
        image_sub = rospy.Subscriber('camera/color/image_raw', 
                                  Image, self.image_callback)
        depth_sub = rospy.Subscriber('camera/depth/image_raw', 
                                  Image, self.depth_callback)
        points_sub = rospy.Subscriber('camera/depth/points', 
                                  PointCloud2, self.point_callback)
        info_sub = rospy.Subscriber('camera/depth/camera_info', 
                                  CameraInfo, self.info_callback)
    #Identify colours based on hue values
    def ColourDetection(self):
        global colour
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
    
    #Identify shapes based on number of edges
    def ShapeDetection(self):
        imgCopy = self.img.copy()
        
        #Convert to HSV
        imgHSV = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        hue_min, hue_max = self.ColourDetection()
        
        #Threshold HSV values
        lower = np.array([hue_min, 1, 220])
        upper = np.array([hue_max, 255, 255])
        mask = cv2.inRange(imgHSV, lower, upper)
        imgResult = cv2.bitwise_and(self.img, self.img, mask=mask)

        imgGray = cv2.cvtColor(imgResult,  cv2.COLOR_BGR2GRAY)
        imgDil = mask.copy()
        
        #Identify the contours of objects in the image
        #Source: https://www.youtube.com/watch?v=WQeoO7MI0Bs&t=2842s
        contours,  hierarchy = cv2.findContours(imgDil,  cv2.RETR_EXTERNAL,  cv2.CHAIN_APPROX_NONE)[-2:]
        for cnt in contours:
            cv2.drawContours(imgDil,  cnt,  -1,  (255, 0, 0), 3)
            peri = cv2.arcLength(cnt,  True)
            approx = cv2.approxPolyDP(cnt,  0.02*peri,  True)
            objCor = len(approx)
            x,  y,  w,  h = cv2.boundingRect(approx)
            
            #Find centroid of contours
            #Source: pyimagesearh.com/2016/02/01/opencv-center-of-contour/
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cX = int(M["m10"]/M["m00"])
                cY = int(M["m01"]/M["m00"])
                objectType = ""
                #Identify objects based on number of edges
                if objCor == 3: objectType = "Triangle"
                elif objCor == 4:
                    aspRatio = w/float(h)
                    if aspRatio > 0.9 and aspRatio < 1.1: objectType = "Cube"
                    else: objectType = "Rect Box"
                elif objCor >= 8:
                    objectType = "Cylinder"
                #cv2.rectangle(img,  (x, y),  (x+w,  y+h),  (0,  255,  0), 2)
                if(shape == objectType or shape == "all"):
                    #Calculate vertical height of objects
                    length = (self.depth_array[cY,cX].astype(float))
                    obj_height = round((1.5-length)*100 ,1)
                    
                    #Calculate global X Y coordinates
                    #Source: http://wiki.ros.org/kinect_node
                    uvw = np.dot(self.R_matrix , self.point_cloud[cX*cY].transpose())
                    x_inertial = round(uvw[0]/uvw[2])*0.001
                    y_inertial= round(uvw[1]/uvw[2])*0.001
                    
                    #Add text to object for testing purposes
                    text1 = "Shape = " + objectType
                    text2 = "Height = " + str(obj_height) + " cm"
                    text3 = "Pos = [" + str(x_inertial) + ", " + str(y_inertial) + "] m"
                    cv2.putText(imgCopy,  text1,  (x+(w/2)+20,  y+(h/2)-10),  cv2.FONT_HERSHEY_COMPLEX,  0.3,  (255, 255, 255),  1)
                    cv2.putText(imgCopy,  text2,  (x+(w/2)+20,  y+(h/2)),  cv2.FONT_HERSHEY_COMPLEX,  0.3,  (255, 255, 255),  1)
                    cv2.putText(imgCopy,  text3,  (x+(w/2)+20,  y+(h/2)+10),  cv2.FONT_HERSHEY_COMPLEX,  0.3,  (255, 255, 255),  1)

        cv2.imshow("window", imgCopy)
        cv2.waitKey(10)
    
    #Callback for the purpose of reading RGB data
    def image_callback(self, img_rgb):
        try:
            self.img = self.bridge.imgmsg_to_cv2(img_rgb,desired_encoding='bgr8')
        except CvBridgeError, e:
            print e

        if(self.depth_array is not None and self.point_cloud is not None and self.R_matrix is not None):
            self.ShapeDetection()
            
    #Callback for the purpose of reading depth data
    def depth_callback(self, msg):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
            self.depth_array = np.array(depth_img, dtype=np.float32)
        except CvBridgeError, e:
            print e 
    
    #Callback for the purpose of reading point cloud data
    def point_callback(self, points):
        point_list = []
        #point_list = list(pc2.read_points(points, skip_nans = True, field_names = ("x", "y", "z")))
        for data in pc2.read_points(points, skip_nans=True):
            point_list.append([data[0], data[1], data[2], data[3]])
        self.point_cloud = np.array(point_list)
        
    #Callback for the purpose of reading the project matrix
    def info_callback(self, info):
        self.R_matrix = np.array(info.P).reshape(3,4)

#Colour and shape identifier
def setVars():
    global colour
    global shape 
    colour = col.get()
    shape = shp.get()

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
    for num in xrange(0,number+1):
        pos_x = random.uniform(-0.5,0.5)
        pos_y = random.uniform(-0.5,0.5)
        item_name = "Object_{0}".format(num)
        item_pose = Pose(Point(x=pos_x, y = pos_y, z = 0), orientation)
        if(num == 1):
            spawn(item_name, obj1, "", item_pose, "world")
        elif(num == 2):
            spawn(item_name, obj3, "", item_pose, "world")
        #elif(num > 2 and num <=4):
            #spawn(item_name, obj3, "", item_pose, "world")
        elif(num == 3):
            spawn(item_name, obj4, "", item_pose, "world")
        time.sleep(0.1)

def close_windows(num, d):
    delete_objects(num, d, True)
    cv2.destroyAllWindows()
    root.destroy()
    quit()

#Used to reset the objects in the kinects field of view
def reset_obj(num, s, d , obj1, obj2, obj3, obj4, orientation):
    delete_objects(num, d, False)
    time.sleep(0.1)
    spawn_objects(num, s,obj1, obj2, obj3, obj4, orientation)


#Block until service is available
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
with open("/home/javad/model_editor_models/kinect_ros/kinect.sdf", "r") as f:
    kinect = f.read()

#Spawn the kinect into the world
item_pose = Pose(Point(x=0, y=0, z = kinect_height), orient_kinect)
s_kinect("kinect1", kinect, "",item_pose, "world")
time.sleep(0.1)

#Spawn the objects for detection
obj_num = 3
spawn_objects(obj_num, s_obj, blue_box, red_box, green_cube, yellow_cylinder, orient_obj)

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
goButton = tk.Button(root, text = "Go!", command = setVars).place(relx = 0.1, rely = 1.0, anchor = 'sw')
deleteButton = tk.Button(root, text = "Exit", command = lambda: close_windows(obj_num, d)).place(relx = 0.9, rely = 1.0, anchor = 'se')
resetButton = tk.Button(root, text = "Reset", command = lambda: reset_obj(obj_num, s_obj, d, blue_box, red_box, green_cube, yellow_cylinder, orient_obj))
resetButton.place(relx = 0.5, rely = 1.0, anchor = 's')

#Small sleep to allow time for the spawning completion - helps with reading kinect data
time.sleep(2)

#Begin detecting objects
obj = ObjectDetection()
rospy.init_node('objectDetection')


root.mainloop()
rospy.spin()



# END ALL

#!/usr/bin/env python
#BEGIN ALL
#Project: Object Detection Module for MTRN4230 Group Project
#Group: G21
#Date: 15/07/2020
#About: The following class controls the object detection of the group 21's project. It uses the information from the kinect sensor to differentiate objects 
#based on colour and shape and then return their global coordinates for use in the trajectory planner class

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


class ObjectDetection:
    def __init__(self, Colour, Shape):
        global colour, shape
        #rospy.init_node("ObjectDetection", anonymous=False)
        self.bridge = cv_bridge.CvBridge() 
        self.img = None
        self.depth_array = None
        self.point_cloud = None
        self.R_matrix = None
        colour = Colour
        shape = Shape
        self.store_vals = False
        self.positions = [None]*10
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
        
        #[height, width] = self.img.shape[1,2]
        
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
        i = 0
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

                    x_min = float(self.point_cloud[:,0].min())
                    x_max = float(self.point_cloud[:,0].max())
                    y_min = float(self.point_cloud[:,1].min())
                    y_max = float(self.point_cloud[:,1].max())

                    cX = float(M["m10"]/M["m00"])
                    cY = float(M["m01"]/M["m00"])

                    y_inertial = -round(x_min +  (x_max*2)*(cX/640), 4)
                    x_inertial = -round(y_min +  (y_max*2)*(cY/480), 4)+0.5
                    if(self.store_vals == True):
                        self.positions[i] = [x_inertial, y_inertial]
                        i = i+1

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
        point_cloud = np.array(list(pc2.read_points(points, skip_nans = True, field_names = ("x", "y", "z"))))
        self.point_cloud = np.array(point_cloud)
        
        
    #Callback for the purpose of reading the project matrix
    def info_callback(self, info):
        self.R_matrix = np.array(info.P).reshape(3,4)
    
    #Return the global coordinates of the objects
    def get_coordinates(self):
        return self.positions

    #Objects being reset
    def reset(self):
        self.store_vals = False
        self.positions = [None]*10

    #Change of colour of shape input
    def UserInput(self, Colour, Shape):
        global colour, shape
        colour = Colour
        shape = Shape
        self.store_vals = True

    def __del__(self):
        print("Closing Object Detection")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        ObjectDetection(colour, shape)
    except KeyboardInterrupt:
        print "Shutting down."

#Begin detecting objects
#obj = ObjectDetection("all", "all")
#rospy.init_node('ObjectDetection')
#rospy.spin()



# END ALL

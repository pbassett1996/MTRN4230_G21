#!/usr/bin/env python
# BEGIN ALL
import rospy, tf
from sensor_msgs.msg import Image
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
import Tkinter as tk
import cv2, cv_bridge, random, time
import numpy as np


colour = "all"
shape = "all"

def empty(a):
    pass

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/color/image_raw', 
                                  Image, self.image_callback)
    def DetermineShape(self,  img):
        imgCopy = img.copy()

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
        #hue_min = cv2.getTrackbarPos("Hue min", "TrackBars")
        #hue_max = cv2.getTrackbarPos("Hue max", "TrackBars")
        #Sat_min = cv2.getTrackbarPos("Sat min", "TrackBars")
        #Sat_max = cv2.getTrackbarPos("Sat max", "TrackBars")
        #Val_min = cv2.getTrackbarPos("Val min", "TrackBars")
        #Val_max = cv2.getTrackbarPos("Val max", "TrackBars")
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower = np.array([hue_min, 1, 220])
        upper = np.array([hue_max, 255, 255])
        mask = cv2.inRange(imgHSV, lower, upper)
        imgResult = cv2.bitwise_and(img, img, mask=mask)

        #imgBlur = cv2.GaussianBlur(imgResult,  (7, 7), 1)
        imgGray = cv2.cvtColor(imgResult,  cv2.COLOR_BGR2GRAY)

        #threshold1 = cv2.getTrackbarPos("Threshold1", "Parameters")
        #threshold2 = cv2.getTrackbarPos("Threshold2", "Parameters")
        #imgCanny = cv2.Canny(imgGray,  threshold1,  threshold2)
        #kernel = np.ones((3,3))
        #imgDil = cv2.dilate(imgCanny, kernel, iterations=1)

        imgDil = mask.copy()

        #cv2.namedWindow("window 2")
        #cv2.imshow("window 2", imgDil)
        #cv2.waitKey(5)

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
        self.DetermineShape(image)


rospy.init_node('follower')
rospy.wait_for_service("gazebo/delete_model")
rospy.wait_for_service("gazebo/spawn_sdf_model")
rospy.wait_for_service("gazebo/spawn_urdf_model")

orient = Quaternion(*tf.transformations.quaternion_from_euler(0,1.57,0))
orient_box = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
s = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
s2 = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
d = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

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

item_name = "kinect1"
d(item_name)
time.sleep(1)
item_pose = Pose(Point(x=0, y=0, z = 1.5), orient)
s("kinect1", kinect, "",item_pose, "world")
follower = Follower()

#for i in xrange(0,5):
for num in xrange(0, 7):
    item_name = "Object_{0}".format(num)
    d(item_name)

for num in xrange(0,7):
    box_x = random.uniform(-0.75,0.75)
    box_y = random.uniform(-0.75,0.75)
    item_name = "Object_{0}".format(num)
    item_pose = Pose(Point(x=box_x, y = box_y, z = 0), orient_box)
    if(num == 1):
        s2(item_name, blue_box, "", item_pose, "world")
    elif(num == 2):
        s2(item_name, red_box, "", item_pose, "world")
    elif(num > 2 and num <=4):
        s2(item_name, green_cube, "", item_pose, "world")
    elif(num > 4 and num <=6):
        s2(item_name, yellow_cylinder, "", item_pose, "world")

def setVars():
    global colour
    global shape 
    colour = col.get()
    shape = shp.get()

root = tk.Tk()
root.title('MTRN4230 - Urm Group Asignment')
root.geometry("400x400")
col = tk.StringVar()
col.set("all")

drop = tk.OptionMenu(root, col, "red", "blue", "green", "yellow", "all")
drop.pack()

shp = tk.StringVar()
shp.set("all")

drop2 = tk.OptionMenu(root, shp, "Rect Box", "Cube", "Cylinder", "all")
drop2.pack()

myButton = tk.Button(root, text = "Go!", command = setVars).pack()
root.mainloop()


rospy.spin()
# END ALL

#!/usr/bin/env python
import numpy as np
import cv2
import rospy
import cv_bridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from time import sleep
import os

class FollowLine:
    def __init__(self):
        # Comportamiento para cuando se interrumpa el programa
        rospy.on_shutdown(self._cleanup)
        # Se crea el objeto cvBridge para traer la imagen del topic
        self.bridge = cv_bridge.CvBridge()
        # Subscriber
        self.image_sub = rospy.Subscriber(
            "video_source/raw", Image, self.image_callback
        ) # video_source/raw - camera/image_raw
        # Publisher
        self.cmd_vel_pub = rospy.Publisher("/line_vel", Twist, queue_size=1)
        self.image_pub = rospy.Publisher("/image_res", Image, queue_size=1)
        # Variable twist para controlar la velocidad del robot
        self.twist = Twist()
        self.image_recieved = np.zeros((0,0))

        self.proc_image = Image()
        
        # PID control
        kp = 0.7
        ki = 1.8
        kd = 0.4
        Ts = 0.05
        K1 = kp + ki*Ts + kd/Ts
        K2 = -kp - 2*kd/Ts
        K3 = kd/Ts

        # Ganancias
        kw = 0.02
        v = 0.13

        cont = 0

        rate = 20
        r = rospy.Rate(rate)
        print("Node initialized at " + str(rate) + "Hz.")

        while not rospy.is_shutdown():
            # Capture the frames
            frame = self.image_recieved

            if frame.any() > 0:
                # Crop the image
                h, w, _ = frame.shape
                
                # Width limits
                limitL = w//2-200
                limitR = w//2+200
                # Height limits
                limitT = h
                limitB = h-50

                # Crop the image
                crop_img = frame[limitB:limitT, limitL:limitR]
                #crop_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2RGB)

                h2, w2, _ = crop_img.shape
                # Convert to grayscale
                gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
                # Gaussian blur
                blur = cv2.GaussianBlur(gray,(5,5),0)
                # Color thresholding
                ret,thresh = cv2.threshold(blur,100,255,cv2.THRESH_BINARY_INV)
                # Find the contours of the frame
                contours,hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)

                # Find the biggest contour (if detected)
                if len(contours) > 0:
                    c = max(contours, key=cv2.contourArea)
                    M = cv2.moments(c)
                    if (M['m00'] > 0):
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])

                        cv2.line(crop_img,(cx,0),(cx,720),(255,0,0),1)
                        cv2.line(crop_img,(0,cy),(1280,cy),(255,0,0),1)

                        cv2.line(crop_img, (w2 - 20, h), (w2 + 20, h), (0, 255, 0), 1)
                
                        cv2.drawContours(crop_img, contours, -1, (0,255,0), 1)
                        
                        # Crop publish
                        self.proc_image = self.bridge.cv2_to_imgmsg(crop_img)
                        self.image_pub.publish(self.proc_image)
                        
                        # Velocity control
                        var = 100
                        # print(cx, w2, w)
                        error = cx - w2//2
                        va = -(kw*error + K1*error + K2*error + K3*error)/100
                        
                        if cx < w2//2 + var and cx > w2//2 - var:
                            #print ("On Track!")
                            self.twist.linear.x = v
                            self.twist.angular.z = 0.0

                        else:
                            #print ("Not on Track!")
                            self.twist.linear.x = v
                            self.twist.angular.z = np.clip(va, -0.8, 0.8)
                        
                        os.system('clear')
                        print(self.twist)
                        print(str(cont))
                        self.cmd_vel_pub.publish(self.twist)

                # elif (len(contours) <= 0 and cont == 0):
                #     print("Going straight!")
                #     self.twist.linear.x = v
                #     self.twist.angular.z = 0.0
                #     self.cmd_vel_pub.publish(self.twist)
                #     cont = 1
                #     sleep(3)
                # elif (len(contours) <= 0 and cont == 1):
                #     print("Turning right!")
                #     self.twist.linear.x = 0.15
                #     self.twist.angular.z = -0.1
                #     self.cmd_vel_pub.publish(self.twist)
                #     cont = 0
                #     sleep(3.5)
                else:
                    print ("I don't see the line")
            
            r.sleep()

    def image_callback(self, msg):
        try:
            self.image_recieved = self.bridge.imgmsg_to_cv2(msg)
        except cv_bridge.CvBridgeError as e:
            print(e)
    
    def _cleanup(self):
        vel_msg = Twist()
        self.cmd_vel_pub.publish(vel_msg)
    
if __name__ == "__main__":
    rospy.init_node("follower", anonymous=True)
    FollowLine()
    rospy.spin()
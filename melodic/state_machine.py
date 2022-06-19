#!/usr/bin/env python
"""
Authors: Oscar D, Carlos H, Juan T
Description: State machine that incorporates both linea follower and image node scripts.
"""
import numpy as np
import cv2
import rospy
import cv_bridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32
import time
import os

class Admin_machine:
    def __init__(self):
        # Comportamiento para cuando se interrumpa el programa
        rospy.on_shutdown(self._cleanup)

        # Subscribers
        self.line_sub = rospy.Subscriber("/line_vel", Twist, self.line_cb) #Suscriptor a la velocidad del nodo de line follower
        self.signal_sub = rospy.Subscriber("/sig_id", Int32, self.signal_cb)
        self.signal_sub = rospy.Subscriber("/sem_id", Int32, self.sem_cb) #Suscriptor a la deteccion de los semaforos

        # Publishers
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # Variable twist para controlar la velocidad del robot
        self.twist = Twist()
        self.line_vel = Twist()

        self.sig_id = -1
        self.sem_id = -1
        counter = 0
        state = 'Follow_line'
        inter = 0
        end_program = False

        rate = 20
        r = rospy.Rate(rate)
        print("Node initialized at " + str(rate) + "Hz.")
        while not rospy.is_shutdown():
            if end_program:
                break

            sig = self.sig_id
            sem = self.sem_id
            print("signal: ", sig, " sem: ", sem)
            
            #######################################################################
            if state == 'Follow_line':
                if sig == 0:
                    if sem == 3:
                        self.twist.angular.z = 0.0
                        self.twist.linear.x = 0.08
                        self.cmd_vel_pub.publish(self.twist)
                        
                        time.sleep(3.0)
                    elif sem == 4:
                        print("Stop")

                        self.twists = Twist()
                        self.cmd_vel_pub.publish(self.twist)

                        while sem == 4:
                            sem = self.sem_id
                            sig = self.sig_id
                        print('Crossing intersection')

                        self.twist.angular.z = 0.0
                        self.twist.linear.x = 0.08
                        self.cmd_vel_pub.publish(self.twist)
                        
                        time.sleep(5.0)
                elif sig == 2:
                    print('Increase_speed')
                    state = 'Increase_speed'
                elif sig == 1:
                    print("Turn_right")
                    state = "Turn_right"

                    if sem == 3:
                        self.twist.angular.z = 0.0
                        self.twist.linear.x = 0.08
                        self.cmd_vel_pub.publish(self.twist)
                        
                        time.sleep(3.5)

                        self.twist.angular.z = -0.1
                        self.twist.linear.x = 0.0 
                        self.cmd_vel_pub.publish(self.twist)

                        time.sleep(3.2)

                        self.twist.angular.z = 0.0
                        self.twist.linear.x = 0.08
                        self.cmd_vel_pub.publish(self.twist)
                        
                        time.sleep(1.5)
                    elif sem == 4:
                        print("Stop")

                        self.twists = Twist()
                        self.cmd_vel_pub.publish(self.twist)

                        while sem == 4:
                            sem = self.sem_id
                            sig = self.sig_id

                        print("Turn right")

                        self.twist.angular.z = 0.0
                        self.twist.linear.x = 0.08
                        self.cmd_vel_pub.publish(self.twist)
                        
                        time.sleep(3.5)

                        self.twist.angular.z = -0.1
                        self.twist.linear.x = 0.0 
                        self.cmd_vel_pub.publish(self.twist)

                        time.sleep(3.2)

                        self.twist.angular.z = 0.0
                        self.twist.linear.x = 0.08
                        self.cmd_vel_pub.publish(self.twist)
                        
                        time.sleep(1.5)
                elif sig == 6:
                    state = 'Stop'
                else:
                    print('Following Line')
                    self.twist = self.line_vel

            #######################################################################
            elif state == 'Increase_speed':
                # if sem == 4:
                #     state = 'Stop'
                if sig == 1:
                    counter = 0
                    state = 'Turn_right'
                else:
                    print('Going faster')
                    self.twist = self.line_vel
                    self.twist.linear.x = 0.15
                    
            #######################################################################
            elif state == 'Stop':
                print('Stopped')
                self.twist.angular.z = 0.0
                self.twist.linear.x = 0.0
                end_program = True
            
            #######################################################################
            else:
                print('No state defined')
                self.twist = self.line_vel

            self.cmd_vel_pub.publish(self.twist)    #Publicamos la velocidad al puzzlebot
            r.sleep()

    def line_cb(self, line_msg):
        self.line_vel = line_msg

    def signal_cb(self, signal_msg):
        self.sig_id = signal_msg.data

    def sem_cb(self, sem_msg):
        self.sem_id = sem_msg.data
    
    def _cleanup(self):
        vel_msg = Twist()
        self.cmd_vel_pub.publish(vel_msg)

if __name__ == "__main__":
    rospy.init_node("follower", anonymous=True)
    Admin_machine()
    # rospy.spin()

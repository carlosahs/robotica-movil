#!/usr/bin/env python3
import numpy as np
import torch
import cv2
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import heapq

SIGNAL_THRESHOLD_HI = 6500
SIGNAL_THRESHOLD_LO = 3000

SEM_THRESHOLD = 1000

class ImageDetect:
    def __init__(self):
        # Comportamiento para cuando se interrumpa el programa
        rospy.on_shutdown(self._cleanup)
        # Se crea el objeto cvBridge para traer la imagen del topic
        self.bridge = cv_bridge.CvBridge()
        # Subscriber
        self.image_sub = rospy.Subscriber(
            "/video_low_res", Image, self.image_callback
        ) # video_source/raw - camera/image_raw - /video_low_res
        # Publisher        
        self.sem_pub = rospy.Publisher("/sem_color", Int32, queue_size=1)
        self.signal_pub = rospy.Publisher("/signal_detected", Int32, queue_size=1) 

        # Variables para guardar la imagen y devolver las señales detectadas
        self.image_recieved = np.zeros((0,0))
        
        rate = 20
        r = rospy.Rate(rate)
        print("Node initialized at " + str(rate) + "Hz.")
        
        model = torch.hub.load('ultralytics/yolov5', 'custom', 
            path='/media/oz26/Oz/Robotics/te3002b/Computer_Vision/Manchester/yolov5/runs/train/exp13/weights/last.pt') 
            #force_reload=True

        signals, semaphores = [], []

        while not rospy.is_shutdown():
            image = self.image_recieved

            if image.any() > 0:
                rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                ####### Codigo pa mandaarle la imagen a la red ##################
                results = model(rgb)

                data = results.pandas().xyxy[0]
                data['area'] = abs(data['xmin'] - data['xmax']) * abs(data['ymin'] - data['ymax'])
                data.drop(columns=['xmax', 'xmin', 'ymax', 'ymin'])

                data = data.to_dict()

                signals, semaphores = self._get_preds_pq(data)

                if len(signals) > 0:
                    # print(data)
                    print(signals[0])

                    if len(signals) > 0 and (
                        -signals[0][0] <= SIGNAL_THRESHOLD_HI and -signals[0][0] >= SIGNAL_THRESHOLD_LO
                    ):
                        self.signal_pub.publish(signals[0][1])
                    else:
                        self.signal_pub.publish(-1)
                else:
                    self.signal_pub.publish(-1)

                if len(semaphores) > 0:
                    print(semaphores[0])

                    if len(semaphores) > 0 and semaphores[0][0] >= SEM_THRESHOLD:
                        self.sem_pub.publish(semaphores[0][1]) # Publish semaphore class
                    else:
                        self.sem_pub.publish(-1)
                else:
                    self.sem_pub.publish(-1)

                # cv2.imshow('YOLO', np.squeeze(results.render())) 
                # if cv2.waitKey(10) & 0xFF == ord('q'):
                #     break
            r.sleep()  
            
        image.release()
        # cv2.destroyAllWindows()

    def _get_preds_pq(self, data):
        num_signals = len(data["class"])
        signal_list = []
        semaphore_list = []
        for i in range(num_signals):
            signal_map = {}
            for key in data.keys():
                datum = data[key][i]
                signal_map[key] = datum
            confidence = signal_map["confidence"]
            name = signal_map["name"]
            name_id = signal_map["class"]
            area = signal_map["area"]
            # area = abs(xmin - xmax) * abs(ymin - ymax)
            if signal_map["name"].find("semaphore") >= 0:
                semaphore_list.append((area, name_id, name, confidence))
            else:
                signal_list.append((-area, name_id, name, confidence))
        heapq.heapify(signal_list)
        heapq.heapify(semaphore_list)
        return signal_list, semaphore_list

    def image_callback(self, msg):
        try:
            self.image_recieved = self.bridge.imgmsg_to_cv2(msg)
        except cv_bridge.CvBridgeError as e:
            print(e)
            
    def _cleanup(self):
        pass
    
if __name__ == "__main__":
    rospy.init_node("prediction", anonymous=True)
    ImageDetect()
    # rospy.spin()

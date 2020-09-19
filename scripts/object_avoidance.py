#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import torch.nn as nn
import torch.nn.functional as F
import torch
import torchvision
import numpy as np
import time


mean = 255.0 * np.array([0.485, 0.456, 0.406])
stdev = 255.0 * np.array([0.229, 0.224, 0.225])

normalize = torchvision.transforms.Normalize(mean, stdev)

model = torchvision.models.alexnet(pretrained=False)
model.classifier[6] = torch.nn.Linear(model.classifier[6].in_features, 2)

model.load_state_dict(torch.load('/home/rg28/jetbot_ws/src/jetbot_sim/scripts/best_model.pth', map_location=lambda storage, loc: storage))
# model.load_state_dict(torch.load('/home/rg28/jetbot_ws/src/jetbot_sim/scripts/best_model.pth'))
# device = torch.device('gpu')
# model = model.to(device)


class object_avoid():
    def __init__(self):
        rospy.Subscriber("camera1/image_raw", Image, self.callback)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.bridge = CvBridge()

       
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        send_vel = self.object_detect(cv_image)
        self.velocity_publisher.publish(send_vel)

    def object_detect (self, cv_image):
        x = self.preprocess(cv_image)
        y = model(x)

        y = F.softmax(y, dim=1)

        prob_blocked = float(y.flatten()[0])
        print(prob_blocked)

        twist = Twist()
        if prob_blocked < 0.12:
            twist.linear.x = 0.5
            twist.angular.z = 0.0
            print("forward")
        else:
            twist.linear.x = 0.0
            twist.angular.z = 1.0
            print("left")
            time.sleep(0.5)

        return twist

    def preprocess(self, cv_image):
        global normalize
        x = cv_image
        x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB)
        x = x.transpose((2, 0, 1))
        x = torch.from_numpy(x).float()
        x = normalize(x)
        # x = x.to(device)
        x = x[None, ...]
        return x

if __name__ == '__main__':
    rospy.init_node('object_avoidance', anonymous=True)
    object_avoid()
    rospy.spin()
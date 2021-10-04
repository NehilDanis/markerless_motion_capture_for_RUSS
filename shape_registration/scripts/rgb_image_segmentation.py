#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from sensor_msgs.msg import Image
import numpy as np
import cv2
import os
import time

import torch
import torchvision.transforms as transforms
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')



class UltrasoundSegmentation():
    def __init__(self):
        PATH = rospy.get_param("segmentation/model_path")
        self.model = torch.load(PATH)
        # Set the model to evaluate model
        self.model.eval()

        self.image_sub = message_filters.Subscriber("/k4a/rgb/image_rect_color", Image)
        self.depth_sub = message_filters.Subscriber("/k4a/depth_to_rgb/image_rect", Image)
        self.cam_info_sub = message_filters.Subscriber("/k4a/depth_to_rgb/camera_info", CameraInfo)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub, self.cam_info_sub], 1, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.callback)

        self.pub = rospy.Publisher("mask", Image, queue_size=1)
        self.pub_img = rospy.Publisher("img", Image, queue_size=1)
        self.pub_depth = rospy.Publisher("depth_img", Image, queue_size=1)
        self.pub_cam_info = rospy.Publisher("cam_info", CameraInfo, queue_size=1)

    def callback(self, img_msg, depth_msg, cam_info_msg):
        img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape((img_msg.height, img_msg.width, -1))

        # segmentation
        img = img[:, :, :3]
        orig_shape = (img.shape[1], img.shape[0])

        temp_img = cv2.resize(img, (int(img.shape[1] / 3), int(img.shape[0] / 3)))
        cv2.imshow("original_img", temp_img)
        cv2.waitKey(1)

        # the image coming from the camera is BGR --> make it RGB
        # change the size to (1, 3, 512, 512)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (512, 512)).transpose(2, 0, 1).reshape(1, 3, 512, 512)

        with torch.no_grad():
            a = self.model(torch.from_numpy(img).type(torch.cuda.FloatTensor) / 255)

        mask = np.array(a.cpu().detach().numpy()[0][0]>0.2) * 1

        img = img.reshape(512, 512, 3)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        img[mask==1] = 255
        img[mask==0] = 0

        img = cv2.resize(img, orig_shape, cv2.INTER_NEAREST)

        largest_contour_mask = np.zeros(img.shape, np.uint8)

        fc = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contours = fc[0]
        largest_contour = max(contours, key=cv2.contourArea)

        cv2.drawContours(largest_contour_mask, [largest_contour], -1, (255,255,255), -1)

        temp_mask = cv2.resize(largest_contour_mask, (int(largest_contour_mask.shape[1] / 3), int(largest_contour_mask.shape[0] / 3)))
        cv2.imshow("mask", temp_mask)
        cv2.waitKey(1)

        mask = largest_contour_mask
        # the calculated mask using the segmentation network is published to the mask topic
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        img_msg.header.stamp = rospy.Time.now()
        depth_msg.header.stamp = rospy.Time.now()
        cam_info_msg.header.stamp = rospy.Time.now()
        msg.height = mask.shape[0]
        msg.width = mask.shape[1]
        msg.encoding = "mono8"
        msg.is_bigendian = False
        msg.step = 1 * mask.shape[1]
        msg.data = np.array(mask).tobytes()
        self.pub.publish(msg)
        self.pub_img.publish(img_msg)
        self.pub_depth.publish(depth_msg)
        self.pub_cam_info.publish(cam_info_msg)


def main(args):
    rospy.init_node('Ultrasound_Segmentation_Node', anonymous=True)
    UltrasoundSegmentation()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

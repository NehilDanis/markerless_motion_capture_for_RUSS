#!/usr/bin/env python2
# license removed for brevity
import rospy
import sys
from sensor_msgs.msg import Image
import numpy as np
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError
import time

import torch
import torchvision.transforms as transforms
from modules.UNet import UNet
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')



class UltrasoundSegmentation():
    def __init__(self):
        PATH = "/home/zhongliang/ros/nehil/markerless_motion_capture_for_RUSS/dataset/UNET/unet_usseg_arm_phantom.pth"
        self.unet = UNet(init_features=64).to(device)
        if self.unet.load_state_dict(torch.load(PATH)): print("ready")
        self.unet.eval()

        self.bridge = CvBridge()
        self.transform_image = transforms.Compose([
                    transforms.ToTensor(),
                    transforms.Normalize((0.5,), (0.5,))
                ])
        self.pub_img = rospy.Publisher("/segmentedImg",Image, queue_size=2)
        self.sub_img = rospy.Subscriber("/ultrasound_img",Image,self.callback)

    def callback(self, img_msg):
        print(device)
        img_msg.encoding = 'mono8'
        try:
          img = self.bridge.imgmsg_to_cv2(img_msg)
        except CvBridgeError as e:
          print(e)

        orig_size = img.shape
        print(orig_size)

        tmp = cv2.resize(img, (256, 256), interpolation=cv2.INTER_LANCZOS4)
        img = tmp.astype(np.float) / 255


        print("started segmentation")

        x = self.transform_image(img)
        x = x.view(-1, 1, 256, 256).to(device, dtype=torch.float)
        pred_tensor = self.unet(x)


        pred = pred_tensor.view(256, 256)


        start_sending_time = time.time()
        pred = pred.cpu().detach().numpy()

        end_sending_time = time.time()
        print("sending the image and receiving back the mask time : ")
        print(end_sending_time - start_sending_time)
        pred = (pred * 255).astype(np.uint8)
        _, mask = cv2.threshold(pred, thresh=127, maxval=255, type=cv2.THRESH_BINARY)

        print("finished segmentation")


#        mask = cv2.resize(mask,(256, 375), interpolation=cv2.INTER_LANCZOS4)
        mask = cv2.resize(mask, (orig_size[1], orig_size[0]), interpolation=cv2.INTER_LANCZOS4)
#        mask = cv2.resize(mask, (375, 550), interpolation=cv2.INTER_LANCZOS4)
#        cv2.imshow("mask", mask)
#        cv2.waitKey(1)



        # the calculated mask using the segmentation network is published to the mask topic
        msg = Image()
        msg.header.stamp = img_msg.header.stamp
        msg.height = mask.shape[0]
        msg.width = mask.shape[1]
        msg.encoding = "mono8"
        msg.is_bigendian = False
        msg.step = 1 * mask.shape[1]
        msg.data = np.array(mask).tobytes()
        self.pub_img.publish(msg)



def main(args):
    rospy.init_node('Ultrasound_Segmentation_Node', anonymous=True)
    UltrasoundSegmentation()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

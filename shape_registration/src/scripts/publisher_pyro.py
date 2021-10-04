#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
#import cv2
from sensor_msgs.msg import Image, CameraInfo
import Pyro5
import message_filters
import Pyro5.api
import numpy as np
import time
#from cv_bridge import CvBridge, CvBridgeError

import msgpack
import msgpack_numpy as m
m.patch()
Pyro5.config.SERIALIZER = "msgpack"




class PublisherPyro(object):
    def __init__(self, uri):
        self.called = False
        self.pyro_server = Pyro5.api.Proxy(uri)
        self.image_sub = message_filters.Subscriber("/k4a/rgb/image_rect_color", Image)
        self.depth_sub = message_filters.Subscriber("/k4a/depth_to_rgb/image_rect", Image)
        self.cam_info_sub = message_filters.Subscriber("/k4a/depth_to_rgb/camera_info", CameraInfo)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub, self.cam_info_sub], 1, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.callback)
        #self.msg_filter_cache = message_filters.Cache(self.ts, 1)
        self.pub = rospy.Publisher("mask", Image, queue_size=1)
        self.pub_img = rospy.Publisher("img", Image, queue_size=1)
        self.pub_depth = rospy.Publisher("depth_img", Image, queue_size=1)
        self.pub_cam_info = rospy.Publisher("cam_info", CameraInfo, queue_size=1)
#        while True:
#            if self.msg_filter_cache.getLastestTime():
#                print(self.msg_filter_cache.getLastestTime())


    def callback(self, img_msg, depth_msg, cam_info_msg):
        self.pyro_server._pyroClaimOwnership()
        #img1 = np.frombuffer(img_msg.data, dtype=np.uint8)
        #print(img1.shape)
        img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape((img_msg.height, img_msg.width, -1))
        #img = np.ones((200, 200))
        #img[:,:50] = np.zeros((200, 50))
        start_sending_time = time.time()
#        print("hello")
        mask = self.pyro_server.segment_arm(img.copy())
#        print("world")
        end_sending_time = time.time()
        print("sending the image and receiving back the mask time : ")
        print(end_sending_time - start_sending_time)

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
    rospy.init_node('Pyro_Publisher_Node', anonymous=True)
    #uri = find_arm_segmentation_server()
    PublisherPyro('PYRONAME:segmentation.arm_segment')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)


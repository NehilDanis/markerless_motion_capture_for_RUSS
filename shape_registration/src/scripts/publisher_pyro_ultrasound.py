#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from sensor_msgs.msg import Image, CameraInfo
import Pyro5
import message_filters
import Pyro5.api
import numpy as np
import time

import msgpack
import msgpack_numpy as m
m.patch()
Pyro5.config.SERIALIZER = "msgpack"

class PublisherPyro(object):
    def __init__(self, uri):
        self.pyro_server = Pyro5.api.Proxy(uri)
#        self.sub_img = rospy.Subscriber("/imfusion/cephasonics",Image,self.callback)
        self.pub_img = rospy.Publisher("segmentedImg",Image, queue_size=1)
        while(True):
            img_msg = rospy.wait_for_message("/imfusion/cephasonics",Image)
            self.callback(img_msg)

    def callback(self, img_msg):
        self.pyro_server._pyroClaimOwnership()
        img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape((img_msg.height, img_msg.width, -1))


        start_sending_time = time.time()
        mask = self.pyro_server.segment_ultrasound(img.copy())
        end_sending_time = time.time()
        print("sending the image and receiving back the mask time : ")
        print(end_sending_time - start_sending_time)
        # the calculated mask using the segmentation network is published to the mask topic
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.height = mask.shape[0]
        msg.width = mask.shape[1]
        msg.encoding = "mono8"
        msg.is_bigendian = False
        msg.step = 1 * mask.shape[1]
        msg.data = np.array(mask).tobytes()
        self.pub_img.publish(msg)


def main(args):
    rospy.init_node('Pyro_Publisher_Node', anonymous=True)
    PublisherPyro('PYRONAME:segmentation.ultrasound_segment')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)


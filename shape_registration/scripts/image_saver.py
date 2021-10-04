#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from sensor_msgs.msg import Image
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError

'''
This file is used to save images into given path using the image_saving_for_training_launch_file
You can also change the topic where the images are read from.
'''

class ImageSaver():
    def __init__(self):
        self.set_id = 1
        self.num_img = 1
        self.count = rospy.get_param("ImageSaverNode/img_index")
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/rgb_image", Image, self.callback)
        self.rate = rospy.Rate(1) # 1 Hz
        self.path = rospy.get_param("ImageSaverNode/path")
        # Do stuff, maybe in a while loop

    def callback(self, img_msg):
        self.rate.sleep() # Sleeps for 1/rate sec
        try:
          cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
          print(e)
        file_name = "img" + str(self.count) + ".jpg"
        cv2.imshow("img", cv_image)
        cv2.waitKey(1)
        cv2.imwrite(os.path.join(self.path, file_name), cv_image)
        self.count += 1

def main(args):
    rospy.init_node('ImageSaverNode', anonymous=True)
    pp = ImageSaver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)


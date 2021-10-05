import torch
import Pyro5
import Pyro5.api

import numpy as np
import cv2

import time

import msgpack_numpy as m
m.patch()
Pyro5.config.SERIALIZER = "msgpack"
Pyro5.config.SERVERTYPE = "multiplex"

@Pyro5.api.expose
@Pyro5.server.behavior(instance_mode="single")
class SegmentArm(object):
    def __init__(self):
        self.sum = 0
        self.idx = 0
        # create the model using the given model_path
        # Load the trained model
        #self.model = torch.load('/home/nehil/arm_w_tarso_data_folder/weights_vgg16.pt')
        self.model = torch.load('/home/zhongliang/ros/nehil/markerless_motion_capture_for_RUSS/tracking_arm/network/weights_vgg16_2.pt')



        # Set the model to evaluate model

        self.model.eval()

    def segment_arm(self, img):

        img = img[:, :, :3]
        orig_shape = (img.shape[1], img.shape[0])
        # print("start arm segmentation")
        # the image coming from the camera is BGR --> make it RGB
        # change the size to (1, 3, 512, 512)

        temp_img = cv2.resize(img, (int(img.shape[1] / 3), int(img.shape[0] / 3)))
        cv2.imshow("original_img", temp_img)
        cv2.waitKey(1)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (512, 512)).transpose(2, 0, 1).reshape(1, 3, 512, 512)
        #print(img.shape)

        start_seg = time.time()
        with torch.no_grad():
            a = self.model(torch.from_numpy(img).type(torch.cuda.FloatTensor) / 255)
        end_seg = time.time()

        start_detach_tim = time.time()
        mask = np.array(a.cpu().detach().numpy()[0][0]>0.2) * 1
        end_detach_tim = time.time()

        #print(end_detach_tim - start_detach_tim)

        img = img.reshape(512, 512, 3)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        #cv2.bitwise_and(img, img, mask = mask)
        img[mask==1] = 255
        img[mask==0] = 0

        img = cv2.resize(img, orig_shape, cv2.INTER_NEAREST)

        largest_contour_mask = np.zeros(img.shape, np.uint8)

        fc = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contours = fc[0]
        largest_contour = max(contours, key=cv2.contourArea)

        cv2.drawContours(largest_contour_mask, [largest_contour], -1, (255,255,255), -1)

        #print(f"Segmentation takes : {end_seg - start_seg}")
        # if self.idx < 500:
        #     self.sum += end_seg - start_seg
        #     self.idx += 1
        # elif self.idx == 500:
        #     print(f"Avg seg time: {self.sum / self.idx}")

        temp_mask = cv2.resize(largest_contour_mask, (int(largest_contour_mask.shape[1] / 3), int(largest_contour_mask.shape[0] / 3)))
        cv2.imshow("mask", temp_mask)
        cv2.waitKey(1)

        return largest_contour_mask


if __name__ == '__main__':
    # arm_segment = SegmentArm()
    # for example purposes we will access the daemon and name server ourselves and not use serveSimple

    daemon = Pyro5.server.Daemon()  # make a Pyro daemon
    ns = Pyro5.api.locate_ns()  # find the name server
    uri = daemon.register(SegmentArm)  # register the greeting maker as a Pyro object
    ns.register("segmentation.arm_segment", uri)  # register the object with a name in the name server

    print("Ready.")
    daemon.requestLoop()

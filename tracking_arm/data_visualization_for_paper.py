# # code for displaying multiple images in one figure
#
# #import libraries
# import cv2
# from matplotlib import pyplot as plt
#
# # create figure
# fig = plt.figure(figsize=(10, 7))
#
# # setting values to rows and column variables
# rows = 2
# columns = 4
#
# # reading images
# Image1 = cv2.imread('/home/nehil/new_arm_w_tarso_data_folder/Train/images/img157.jpg')
# Image2 = cv2.imread('/home/nehil/new_arm_w_tarso_data_folder/Train/images/img183.jpg')
# Image3 = cv2.imread('/home/nehil/new_arm_w_tarso_data_folder/Train/images/img405.jpg')
# Image4 = cv2.imread('/home/nehil/new_arm_w_tarso_data_folder/Train/images/img721.jpg')
#
# Mask1 = cv2.imread('/home/nehil/new_arm_w_tarso_data_folder/Train/masks/mask157.jpg')
# Mask2 = cv2.imread('/home/nehil/new_arm_w_tarso_data_folder/Train/masks/mask183.jpg')
# Mask3 = cv2.imread('/home/nehil/new_arm_w_tarso_data_folder/Train/masks/mask405.jpg')
# Mask4 = cv2.imread('/home/nehil/new_arm_w_tarso_data_folder/Train/masks/mask721.jpg')
#
#
# # Adds a subplot at the 1st position
# fig.add_subplot(rows, columns, 1)
#
# # showing image
# plt.imshow(Image1)
# plt.axis('off')
# plt.title("Image 1")
#
# # Adds a subplot at the 2nd position
# fig.add_subplot(rows, columns, 2)
#
# # showing image
# plt.imshow(Image2)
# plt.axis('off')
# plt.title("Image 2")
#
# # Adds a subplot at the 3rd position
# fig.add_subplot(rows, columns, 3)
#
# # showing image
# plt.imshow(Image3)
# plt.axis('off')
# plt.title("Image 3")
#
# # Adds a subplot at the 4th position
# fig.add_subplot(rows, columns, 4)
#
# # showing image
# plt.imshow(Image4)
# plt.axis('off')
# plt.title("Image 4")
#
# # Adds a subplot at the 1st position
# fig.add_subplot(rows, columns, 5)
#
# # showing image
# plt.imshow(Mask1)
# plt.axis('off')
# plt.title("Mask 1")
#
# # Adds a subplot at the 2nd position
# fig.add_subplot(rows, columns, 6)
#
# # showing image
# plt.imshow(Mask2)
# plt.axis('off')
# plt.title("Mask 2")
#
# # Adds a subplot at the 3rd position
# fig.add_subplot(rows, columns, 7)
#
# # showing image
# plt.imshow(Mask3)
# plt.axis('off')
# plt.title("Mask 3")
#
# # Adds a subplot at the 4th position
# fig.add_subplot(rows, columns, 8)
#
# # showing image
# plt.imshow(Mask4)
# plt.axis('off')
# plt.title("Mask 4")
#
# plt.show()

import open3d as o3d

pcd = o3d.io.read_point_cloud("/home/nehil/catkin_ws_registration/src/artery_downsampled_robot_base.pcd")

file1 = open("/home/nehil/Desktop/artery.txt","w")
L = []
for point in pcd.points:
    L.append(str(point[0]) + ", " + str(point[1]) + ", " + str(point[2]) + "\n")

file1.writelines(L)
file1.close()



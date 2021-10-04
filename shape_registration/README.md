# shape_registration
Shape registration between MRI data and the rgbd data from Azure Kinect

![alt text](https://github.com/NehilDanis/shape_registration/raw/master/blob/registration_result.png "registration example")

lookuptransform : to get access to the latest available transforms in that tf2 tree, without knowing at what time that transform was recorded


/image --> the topic where your camera or depth sensor publishes the raw image
/camera_info --> the camera info of your camera or depth sensor, the info of the frame which produces above image (in my case it is the camera info of rgb frame)

image_is_rectified : if the topic you used for the '/image' parameter is produces rectified parameters, then you should set this to true, else to false. This way, the distortion parameters will be set as 0 or as the numbers given in the /camera_info topic

marker_size : the size of the aruco marker you have been using. One edge of a square shaped aruco marker. (double)

marker_id : the number which has been used to create aruco marker (int)

reference_frame : parent frame, like the world frame

camera_frame : child frame 1

marker_frame: child frame 2

Once the image is received from the camera, the marker with the given marker id will be selected and the transformation of the marker wrt thhe camera frame will be found. Later on the transformation from camera to the reference frame will be calculated, if they are equal it will be just identity matrix. Then the righttoleft transformation will be found which is about handling cartesian offset between stereo pairs, see the sensor_msgs/CameraInfo documentation for details. Using all of these, the transformation from the marker coordinate to the reference coordinate will be found.

The transformation will be sent to tf, and also it will be published in the topic called pose.

topic position : shows the translation of the marker wrt ref frame.


pixel_pub : show the center of the marker in the marker frame.

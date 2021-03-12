#!/usr/bin/env python
#!/home/mlg-lin/anaconda2/bin/python
#
# This script in compatible with Object Localization versions: _v196
# This Python file uses the following encoding: utf-8
#
# Just install whatever deep learrning library you want without conda and
# specifiy in the shebang of your ros node what version of python you are using
# (python or python3).
#
# http://projectsfromtech.blogspot.com/2017/10/visual-object-recognition-in-ros-using.html
# http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
# https://answers.ros.org/question/203782/rviz-marker-line_strip-is-not-displayed/

# For errors from TF with graphs and threads:
# Final answer used:
# https://stackoverflow.com/questions/56511483/valueerror-tensor-tensordense-1-sigmoid0-shape-1-dtype-float32-is-n
# Class wrap:
# https://stackoverflow.com/questions/51127344/tensor-is-not-an-element-of-this-graph-deploying-keras-model
# Using single thread:
# https://stackoverflow.com/questions/38187808/how-can-i-run-tensorflow-on-one-single-core

#PC only
import tensorflow as tf
config = tf.compat.v1.ConfigProto(log_device_placement=True)
config.gpu_options.allow_growth = True
#tf.debugging.set_log_device_placement(True)
MySess = tf.compat.v1.Session(config=config)
import sys
# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")
# Import utilites
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

#tf.reset_default_graph()

import cv2
import time
## append back in order to import rospy
#sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import glob
import numpy as np
import math
# ~ # Hope you don't be imprisoned by legacy Python code :)
# ~ from pathlib import Path
# ~ # current working directory is at workspace ros level
# ~ cwd = Path.cwd()
# ~ print(cwd)
import os
dir_path = os.path.dirname(__file__)
# ~ print('##############################')
# ~ print(dir_path)


import rospy
import roslib
from cv_bridge import CvBridge, CvBridgeError
#from std_msgs.msg import String, Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
#from visualization_msgs.msg import Marker
#from geometry_msgs.msg import Point, Pose


#IOU_THRESHOLD = 0.5
#SCORE_THRESHOLD = 0.2 #0.2
#MAX_OUTPUT_SIZE = 1 #49


# Path to frozen detection graph .pb file, which contains the model that is used
# for object detection.
#PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,'frozen_inference_graph.pb')
#PATH_TO_CKPT = '/home/mlg/src/Bote_v1_2019/src/ssd_grasper/src/models/frozen_inference_graph.pb'
PATH_TO_CKPT = dir_path + '/models/frozen_inference_graph.pb'

# Path to label map file
#PATH_TO_LABELS = os.path.join(CWD_PATH,'training','labelmap.pbtxt')
#PATH_TO_LABELS = '/home/mlg/src/Bote_v1_2019/src/ssd_grasper/src/label_map.pbtxt'
PATH_TO_LABELS = dir_path + '/models/label_map.pbtxt'

# Path to image
#PATH_TO_IMAGE = os.path.join(CWD_PATH,IMAGE_NAME)
#PATH_TO_IMAGE = 'test_images/test_1.jpg'

# Number of classes the object detector can identify
NUM_CLASSES = 1

# Load the label map.
# Label maps map indices to category names, so that when our convolution
# network predicts `5`, we know that this corresponds to `king`.
# Here we use internal utility functions, but anything that returns a
# dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Load the label map.
# Label maps map indices to category names, so that when our convolution
# network predicts `5`, we know that this corresponds to `king`.
# Here we use internal utility functions, but anything that returns a
# dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Load the Tensorflow model into memory.
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    sess = tf.Session(graph=detection_graph)

# Define input and output tensors (i.e. data) for the object detection classifier

# Input tensor is the image
image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

# Output tensors are the detection boxes, scores, and classes
# Each box represents a part of the image where a particular object was detected
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

# Each score represents level of confidence for each of the objects.
# The score is shown on the result image, together with the class label.
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

# Number of objects detected
num_detections = detection_graph.get_tensor_by_name('num_detections:0')


# Normalized center for tracking
Yc, Xc, H, W, Score = 0, 0, 0, 0, 0
TrackCount = 0
YC_LIMIT = 100/float(360)
XC_LIMIT = 100/float(640)

#with tf.device('/gpu:0'):
#MyGraph = tf.compat.v1.get_default_graph()


rospy.init_node('CNN_Grasper', anonymous=True)
MyPub = rospy.Publisher('obj_loc_grasper', Float32MultiArray, queue_size = 1)
MyBridge = CvBridge()
MyMsg = Float32MultiArray()
#MyPredCount = 0


def Img_CenteredCrop(img, outW, outH):
    # Crops an image considering that out.size < img.size
    height, width, depth = img.shape

    tpX = int( (width - outW) / 2 )
    tpY = int( (height - outH) / 2 ) + 60 #60px offset to get bottom ROI
    cropped_img = img[tpY:tpY + int(outH), tpX:tpX + int(outW)]
    return cropped_img


def ROS_OnImage(img_msg):
    global MyBridge

    print('OnImage...')

    # First convert the image to OpenCV image
    image = MyBridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")

    # Center-crop image from Bebop 2 (856x480)->(640x360)
    image = Img_CenteredCrop(image, 640, 360)
    
    #img = cv2.resize(img, (IMAGE_SIZE, IMAGE_SIZE))
    #feat_scaled = preprocess_input(np.array(img, dtype=np.float32))
    
    # Load image using OpenCV and
    # expand image dimensions to have shape: [1, None, None, 3]
    # i.e. a single-column array, where each item in the column has the pixel RGB value
    #image = cv2.imread(img)
    image_expanded = np.expand_dims(image, axis=0)

    # Perform the actual detection by running the model with the image as input
    (boxes, scores, classes, num) = sess.run(
        [detection_boxes, detection_scores, detection_classes, num_detections],
        feed_dict={image_tensor: image_expanded})

    # Draw the results of the detection (aka 'visulaize the results')
    #image.setflags(write=1) # does not work here
    # ~ image_cv = np.copy(image)
    # ~ vis_util.visualize_boxes_and_labels_on_image_array(
        # ~ image_cv,
        # ~ np.squeeze(boxes),
        # ~ np.squeeze(classes).astype(np.int32),
        # ~ np.squeeze(scores),
        # ~ category_index,
        # ~ use_normalized_coordinates=True,
        # ~ line_thickness=3,
        # ~ min_score_thresh=0.95)

    # All the results have been drawn on image. Now display the image.
    # ~ cv2.imshow('ObjectDetector', image_cv)
    # ~ cv2.waitKey(1)

    boxes = np.squeeze(boxes)
    scores = np.squeeze(scores)
    # ~ print(boxes[0])
    # ~ print(scores[0])
    # ~ print('num:', num[0])


    # find the best box for tracking
    global TrackCount, Yc, Xc, H, W, Score
    #MyMsg.data = [Yc, Xc, H, W, Score]
    TrackCount = TrackCount + 1
    for i in range(10):
        y0, x0, y1, x1 = boxes[i][0], boxes[i][1], boxes[i][2], boxes[i][3]
        h = y1 - y0
        w = x1 - x0
        y_c = y0 + h/2
        x_c = x0 + w/2
        
        if abs(Xc - x_c) < XC_LIMIT and abs(Yc - y_c) < YC_LIMIT:
            Yc, Xc, H, W, Score = y_c, x_c, h, w, scores[i]
            #MyMsg.data = [Yc, Xc, H, W, Score]
            TrackCount = 0
            break
            
    if TrackCount > 9:
        # TODO: select by area or width and high
        if scores[0] > 0.95:
            y0, x0, y1, x1 = boxes[0][0], boxes[0][1], boxes[0][2], boxes[0][3]
            h = y1 - y0
            w = x1 - x0
            y_c = y0 + h/2
            x_c = x0 + w/2
            Yc, Xc, H, W, Score = y_c, x_c, h, w, scores[0]
            TrackCount = 0
        else:
            TrackCount = 10
            Yc, Xc, H, W, Score = 0, 0, 0, 0, 0

    MyMsg.data = [Yc, Xc, H, W, Score]
    # if scores[0] > 0.90:
        # y0, x0, y1, x1 = boxes[0][0], boxes[0][1], boxes[0][2], boxes[0][3]
        # h = y1 - y0
        # w = x1 - x0
        # y_c = y0 + h/2
        # x_c = x0 + w/2
        # MyMsg.data = [ y_c, x_c, h, w, scores[0] ]
    # else:
        # MyMsg.data = [0, 0, 0, 0, 0]
    print(MyMsg.data)
    MyPub.publish(MyMsg)


print('Running SSD Grasper...')
#rospy.Subscriber("/ardrone/bottom/image_raw", Image, ROS_OnImage, queue_size = 1, buff_size = 16777216)
#rospy.Subscriber("/ardrone/bottom/image_raw", Image, ROS_OnImage)

def ROS_Listener():
    #while not rospy.is_shutdown():
    #MyPub.publish(MyLineStrip)
    #rospy.sleep(0.5)
    #rospy.spin()
    while not rospy.is_shutdown():
        #msg = rospy.wait_for_message('/ardrone/bottom/image_raw', Image) # tum_simulator
        msg = rospy.wait_for_message('/bebop/image_raw', Image) # Bebop 2
        time1 = time.time() #time.clock()
        ROS_OnImage(msg)
        time2 = time.time() #time.clock()
        print('ImageCallback:', time2-time1)

if __name__ == '__main__':
    ROS_Listener()
    #cv2.destroyAllWindows()

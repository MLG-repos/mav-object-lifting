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

#tf.reset_default_graph()

from tensorflow.keras import Model
from tensorflow.keras.applications.mobilenet_v2 import MobileNetV2, preprocess_input
#from tensorflow.keras.callbacks import ModelCheckpoint, EarlyStopping, ReduceLROnPlateau, Callback, CSVLogger
from tensorflow.keras.layers import Conv2D, BatchNormalization, Activation, concatenate
#from tensorflow.keras.losses import binary_crossentropy
#from tensorflow.keras.regularizers import l2
#from tensorflow.keras.utils import Sequence
#from tensorflow.keras.optimizers import SGD
#from tensorflow.keras.backend import epsilon
#from tensorflow.keras.models import model_from_json
#from tensorflow.keras.optimizers import Adam
#from tensorflow.keras import backend as K

#from train_v1x import create_model, IMAGE_SIZE
#from train_v1x import *

## in order to import cv2 under python3
#import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import time
## append back in order to import rospy
#sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import glob
import numpy as np
import math

import rospy
import roslib
from cv_bridge import CvBridge, CvBridgeError
#from std_msgs.msg import String, Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
#from visualization_msgs.msg import Marker
#from geometry_msgs.msg import Point, Pose


FOLDER = '/home/mlg/src/Bote_v1_2019/src/obj_loc/src/'
WEIGHTS_FILE = FOLDER + 'custom_trained_weights_197_091.h5'
#WEIGHTS_FILE = "custom_trained_weights_0.92.h5"

IMAGE_SIZE = 224
# 0.35, 0.5, 0.75, 1.0
ALPHA = 1.0

#IOU_THRESHOLD = 0.5
SCORE_THRESHOLD = 0.2 #0.2
#MAX_OUTPUT_SIZE = 1 #49


# Normalized center for tracking
Yc, Xc, H, W, Q0, Q1, Q2, Q3, SF, Score = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
TrackCount = 0
YC_LIMIT = 100/float(360)
XC_LIMIT = 100/float(640)


def CreateModel(trainable=False):
    with tf.device('/gpu:0'):
        model = MobileNetV2(input_shape=(IMAGE_SIZE, IMAGE_SIZE, 3), include_top=False, alpha=ALPHA, weights="imagenet")

        for layer in model.layers:
            layer.trainable = trainable

        block = model.get_layer("block_16_project_BN").output # (None, 7, 7, ?), ?=112,320 -> Alpha=0.35,1.0
        x1_ = Conv2D(160, padding="same", kernel_size=3, strides=1, activation="relu")(block)
        x1_ = Conv2D(160, padding="same", kernel_size=3, strides=1, use_bias=False)(x1_)
        x1_ = BatchNormalization()(x1_)
        x1_ = Activation("relu")(x1_)
        #x = Conv2D(5, padding="same", kernel_size=1, activation="sigmoid", name="coords_out")(x)
        x1 = Conv2D(5, padding="same", kernel_size=1, activation="sigmoid", name="coords_out")(x1_)

        #block_q = model.get_layer("block_14_depthwise_BN").output # (None, 7, 7, ?), ?=960 -> Alpha=1.0
        #cls1_fc1_pose = Dense(1024, activation='relu', name='cls1_fc1_pose')(block_low)
        #cls1_fc_pose_wpqr = Dense(4, activation="tanh", name='cls1_fc_pose_wpqr')(cls1_fc1_pose)

        #x = Conv2D(1024, padding="same", kernel_size=3, strides=1, activation="relu")(block)
        #x = Conv2D(1024, padding="same", kernel_size=3, strides=1, use_bias=False)(x)
        #x = BatchNormalization()(x)
        #x = Activation("relu")(x)
        #x = Conv2D(9, padding="same", kernel_size=1, activation="tanh", name="coords_out")(x)

        x2_ = Conv2D(160, padding="same", kernel_size=3, strides=1, use_bias="relu")(block)
        x2_ = Conv2D(160, padding="same", kernel_size=3, strides=1, use_bias=False)(x2_)
        x2_ = BatchNormalization()(x2_)
        x2_ = Activation("relu")(x2_)
        x2 = Conv2D(4, padding="same", kernel_size=1, activation="tanh", name="pose_out")(x2_)

        #x3 = Conv2D(1024, padding="same", kernel_size=3, strides=1, use_bias=False)(block_low)
        #x3 = BatchNormalization()(x3)
        #x3 = Activation("relu")(x3)
        block_sf = model.get_layer("block_15_project_BN").output # (None, 7, 7, ?), ?=960 -> Alpha=1.0
        x3 = Conv2D(160, padding="same", kernel_size=3, strides=1, activation="relu")(block_sf)
        x3 = Conv2D(160, padding="same", kernel_size=3, strides=1, use_bias=False)(x3)
        x3 = BatchNormalization()(x3)
        x3 = Activation("relu")(x3)
        x3 = Conv2D(1, padding="same", kernel_size=1, activation="tanh", name="sf_out")(x3)

        x = concatenate([x1, x2, x3], axis=3, name='conc_out')

        model = Model(inputs=model.input, outputs=x)

        # divide by 2 since d/dweight learning_rate * weight^2 = 2 * learning_rate * weight
        # see https://arxiv.org/pdf/1711.05101.pdf
        #regularizer = l2(WEIGHT_DECAY / 2)
        #for weight in model.trainable_weights:
        #    with tf.keras.backend.name_scope("weight_regularizer"):
        #        model.add_loss(regularizer(weight)) # in tf2.0: lambda: regularizer(weight)

    return model


#SessionConf = tf.ConfigProto(
#      intra_op_parallelism_threads=1,
#      inter_op_parallelism_threads=1)
#MySess = tf.compat.v1.Session(config=SessionConf)

#with tf.device('/gpu:0'):
MyModel = CreateModel()
MyModel.load_weights(WEIGHTS_FILE)
MyGraph = tf.compat.v1.get_default_graph()

rospy.init_node('neural_network', anonymous=True)
MyPub = rospy.Publisher('obj_loc', Float32MultiArray, queue_size = 1)
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
    global MyModel
    global MyBridge

    print('OnImage...')

    # First convert the image to OpenCV image
    img = MyBridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")

    # Center-crop image from Bebop 2 (856x480)->(640x360)
    img = Img_CenteredCrop(img, 640, 360)
    
    img = cv2.resize(img, (IMAGE_SIZE, IMAGE_SIZE))
    feat_scaled = preprocess_input(np.array(img, dtype=np.float32))

    #pred = np.squeeze(model.predict(feat_scaled[np.newaxis,:]))
    #pred = None
    #with MyGraph.as_default():
    time1 = time.time() #time.clock()
    pred = MyModel.predict(feat_scaled[np.newaxis,:])
    time2 = time.time() #time.clock()
    print('Prediccion:', time2-time1)
    pred = np.squeeze(pred)
    #region = np.squeeze(pred[0])
    #pose = np.squeeze(pred[1])
    #height, width, y_f, x_f, score = [a.flatten() for a in np.split(pred, pred.shape[-1], axis=-1)]
    height, width, y_f, x_f, score, p0, p1, p2, p3, sf = \
      [a.flatten() for a in np.split(pred, pred.shape[-1], axis=-1)]
    #p0, p1, p2, p3 = [a.flatten() for a in np.split(pose, pose.shape[-1], axis=-1)]
    #print(pose.shape)

    coords = np.arange(pred.shape[0] * pred.shape[1])
    y = (y_f + coords // pred.shape[0]) / pred.shape[0]
    x = (x_f + coords % pred.shape[1]) / pred.shape[1]

    #boxes = np.stack([y, x, height, width, score], axis=-1)
    boxes = np.stack([y, x, height, width, p0, p1, p2, p3, sf, score], axis=-1)
    boxes = boxes[np.where(boxes[...,-1] >= SCORE_THRESHOLD)]
    selected_index = -1
    if(boxes.shape[0] > 0):
        selected_index = np.argmax(boxes[...,-1]) # Only the first occurrence is returned
        #selected_indices = np.argmax(boxes, axis=-1) # OJO this returns an *array* if called
        #print("Selected index:")
        #print(selected_index)

    #poses = np.stack([p0, p1, p2, p3], axis=-1)

    #selected_indices = tf.image.non_max_suppression(boxes[...,:-1], boxes[...,-1], MAX_OUTPUT_SIZE, IOU_THRESHOLD)
    #selected_indices = tf.image.non_max_suppression(boxes[...,:4], boxes[...,-1], MAX_OUTPUT_SIZE, IOU_THRESHOLD)
    #selected_indices = tf.compat.v1.Session().run(selected_indices)
    #time1 = time.clock()
    #selected_indices = MySess.run(selected_indices)
    #time2 = time.clock()
    #print('MySessRun:', time2-time1)
    
    # indices are greedily selected as a subset of bounding boxes in descending order of score
    if selected_index > -1:
        #y_c, x_c, h, w, q0, q1, q2, q3, sf, score = boxes[selected_indices[0]]
        y_c, x_c, h, w, q0, q1, q2, q3, sf, score = boxes[selected_index]
    else:
        y_c, x_c, h, w, q0, q1, q2, q3, sf, score = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

    # tracking
    global TrackCount, Yc, Xc, H, W, Q0, Q1, Q2, Q3, SF, Score
    TrackCount = TrackCount + 1
    
    errorX = abs(Xc - x_c)
    errorY = abs(Yc - y_c)
    #print('Error Xc: ', errorX, ' Error Yc: ', errorY)
    if errorX < XC_LIMIT and errorY < YC_LIMIT:
        Yc, Xc, H, W, Q0, Q1, Q2, Q3, SF, Score = y_c, x_c, h, w, q0, q1, q2, q3, sf, score
        TrackCount = 0
    
    if TrackCount > 9:
        # TODO: select by area or width and high
        if selected_index > -1:
            Yc, Xc, H, W, Q0, Q1, Q2, Q3, SF, Score = boxes[selected_index]
            TrackCount = 0
        else:
            TrackCount = 10
            Yc, Xc, H, W, Q0, Q1, Q2, Q3, SF, Score = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
            
    MyMsg.data = [Yc, Xc, H, W, Q0, Q1, Q2, Q3, SF, Score]
    print(MyMsg.data)
    MyPub.publish(MyMsg)


print('Running ObjLoc...')
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

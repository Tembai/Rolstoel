#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
import time
from absl import app, flags, logging
from absl.flags import FLAGS
import core.utils as utils
from core.yolov4 import YOLOv4, YOLOv3, YOLOv3_tiny, decode
from PIL import Image
from core.config import cfg
import cv2
import numpy as np
import tensorflow as tf
import pyrealsense2 as rs
import statistics
import os

flags.DEFINE_string('framework', 'tf', '(tf, tflite')
flags.DEFINE_string('weights', './data/yolov4.weights',
                    'path to weights file')
flags.DEFINE_integer('size', 608, 'resize images to')
flags.DEFINE_boolean('tiny', False, 'yolo or yolo-tiny')
flags.DEFINE_string('model', 'yolov4', 'yolov3 or yolov4')

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

PHONE_SIZE = (0.06, 0.01, 0.13)
BOOK_SIZE = (0.11, 0.01, 0.15)

def main(_argv):
    pass

if __name__=="__main__":
    try:
        app.run(main)
    except SystemExit:
        pass

    os.environ["CUDA_VISIBLE_DEVICES"] = "0"
    if FLAGS.tiny:
        STRIDES = np.array(cfg.YOLO.STRIDES_TINY)
        ANCHORS = utils.get_anchors(cfg.YOLO.ANCHORS_TINY, FLAGS.tiny)
    else:
        STRIDES = np.array(cfg.YOLO.STRIDES)
        if FLAGS.model == 'yolov4':
            ANCHORS = utils.get_anchors(cfg.YOLO.ANCHORS, FLAGS.tiny)
        else:
            ANCHORS = utils.get_anchors(cfg.YOLO.ANCHORS_V3, FLAGS.tiny)
    NUM_CLASS = len(utils.read_class_names(cfg.YOLO.CLASSES))
    XYSCALE = cfg.YOLO.XYSCALE
    input_size = FLAGS.size

    if FLAGS.framework == 'tf':
        input_layer = tf.keras.layers.Input([input_size, input_size, 3])
        if FLAGS.tiny:
            feature_maps = YOLOv3_tiny(input_layer, NUM_CLASS)
            bbox_tensors = []
            for i, fm in enumerate(feature_maps):
                bbox_tensor = decode(fm, NUM_CLASS, i)
                bbox_tensors.append(bbox_tensor)
            model = tf.keras.Model(input_layer, bbox_tensors)
            utils.load_weights_tiny(model, FLAGS.weights)
        else:
            if FLAGS.model == 'yolov3':
                feature_maps = YOLOv3(input_layer, NUM_CLASS)
                bbox_tensors = []
                for i, fm in enumerate(feature_maps):
                    bbox_tensor = decode(fm, NUM_CLASS, i)
                    bbox_tensors.append(bbox_tensor)
                model = tf.keras.Model(input_layer, bbox_tensors)
                utils.load_weights_v3(model, FLAGS.weights)
            elif FLAGS.model == 'yolov4':
                feature_maps = YOLOv4(input_layer, NUM_CLASS)
                bbox_tensors = []
                for i, fm in enumerate(feature_maps):
                    bbox_tensor = decode(fm, NUM_CLASS, i)
                    bbox_tensors.append(bbox_tensor)
                model = tf.keras.Model(input_layer, bbox_tensors)
                utils.load_weights(model, FLAGS.weights)

        model.summary()
    else:
        # Load TFLite model and allocate tensors.
        interpreter = tf.lite.Interpreter(model_path=FLAGS.weights)
        interpreter.allocate_tensors()
        # Get input and output tensors.
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        print(input_details)
        print(output_details)

    publisher = rospy.Publisher("object_detection", MarkerArray, queue_size=100)
    rospy.init_node("visualization")

    # create a marker for the camera
    camera = Marker()
    camera.header.frame_id = "base_link"
    camera.type = Marker.CUBE
    camera.pose.position.x = 0
    camera.pose.position.y = 0
    camera.pose.position.z = 0
    camera.scale.x = 0.09
    camera.scale.y = 0.025
    camera.scale.z = 0.025
    camera.color.r = 0.8
    camera.color.g = 0.8
    camera.color.b = 0.8
    camera.color.a = 1
    camera.pose.orientation.w = 1.0

    rate = rospy.Rate(10) # hz
    while not rospy.is_shutdown():
        markers = MarkerArray()
        markers.markers.append(camera)

        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        frame = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(frame)

        frame_size = frame.shape[:2]
        image_data = utils.image_preprocess(np.copy(frame), [input_size, input_size])
        image_data = image_data[np.newaxis, ...].astype(np.float32)
        prev_time = time.time()

        scaled_depth=cv2.convertScaleAbs(depth_image, alpha=0.08)
        depth_colormap = cv2.applyColorMap(scaled_depth, cv2.COLORMAP_JET)

        if FLAGS.framework == 'tf':
            pred_bbox = model.predict(image_data)
        else:
            interpreter.set_tensor(input_details[0]['index'], image_data)
            interpreter.invoke()
            pred_bbox = [interpreter.get_tensor(output_details[i]['index']) for i in range(len(output_details))]

        if FLAGS.model == 'yolov4':
            pred_bbox = utils.postprocess_bbbox(pred_bbox, ANCHORS, STRIDES, XYSCALE)
        else:
            pred_bbox = utils.postprocess_bbbox(pred_bbox, ANCHORS, STRIDES)

        bboxes = utils.postprocess_boxes(pred_bbox, frame_size, input_size, 0.25)
        bboxes = utils.nms(bboxes, 0.213, method='nms')


        for box in bboxes:
            x_mid = int((box[0]+box[2])/2)
            y_mid = int((box[1]+box[3])/2)
            pixel_depths = []
            for i in range(3):
                for j in range(3):
                    pixel_depths.append(depth_frame.get_distance(int(x_mid+i-1),int(y_mid+j-1)))
            object_depth = statistics.median(pixel_depths)
            object_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x_mid, y_mid], object_depth)
            if box[5] == 67.0:
                print('found phone')
                if object_depth == 0.0:
                    print('depth not found')
                else:
                    depth_colormap[max(0, min(y_mid, 479)), max(0, min(x_mid, 639))] = [0,255,0]
                    # create a marker for the phone
                    phone = Marker()
                    phone.header.frame_id = "base_link"
                    phone.type = Marker.CUBE
                    phone.pose.position.x = object_point[0]
                    phone.pose.position.y = object_point[2]
                    phone.pose.position.z = -object_point[1]
                    phone.scale.x = PHONE_SIZE[0]
                    phone.scale.y = PHONE_SIZE[1]
                    phone.scale.z = PHONE_SIZE[2]
                    phone.color.r = 0.1
                    phone.color.g = 0.1
                    phone.color.b = 0.1
                    phone.color.a = 1
                    phone.pose.orientation.w = 1.0
                    markers.markers.append(phone)
            elif box[5] == 73.0:
                print('found book')
                if object_depth == 0.0:
                    print('depth not found')
                else:
                    depth_colormap[max(0, min(y_mid, 479)), max(0, min(x_mid, 639))] = [0,255,0]
                    # create a marker for the phone
                    book = Marker()
                    book.header.frame_id = "base_link"
                    book.type = Marker.CUBE
                    book.pose.position.x = object_point[0]
                    book.pose.position.y = object_point[2]
                    book.pose.position.z = -object_point[1]
                    book.scale.x = BOOK_SIZE[0]
                    book.scale.y = BOOK_SIZE[1]
                    book.scale.z = BOOK_SIZE[2]
                    book.color.r = 0.1
                    book.color.g = 0.5
                    book.color.b = 0.1
                    book.color.a = 1
                    book.pose.orientation.w = 1.0
                    markers.markers.append(book)

        cv2.namedWindow("result", cv2.WINDOW_AUTOSIZE)
        image_color = utils.draw_bbox(frame, bboxes)
        result = cv2.cvtColor(image_color, cv2.COLOR_RGB2BGR)
        image_depth = utils.draw_bbox(depth_colormap, bboxes)
        cv2.imshow("result", image_depth)
        print('-----')
        if cv2.waitKey(1) & 0xFF == ord('q'):
            pipeline.stop()
            break

        index = 0
        for m in markers.markers:
            m.id = index
            index += 1

        publisher.publish(markers)
        rate.sleep()


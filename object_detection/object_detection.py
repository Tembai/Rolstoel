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

# Prepares the alignment of the depth image to the color image
align_to = rs.stream.color
align = rs.align(align_to)

'''
Assuming the camera hasn't been rotated:
A positive x means going to the right of the camera.
A positive y means going in the direction the camera is facing.
A positive z means going upwards.
'''
OBJECT_DEPTH_ESTIMATE = 0.10  # estimate for how deep an object is, since this isn't visible
CAMERA_DISPLACEMENT = [0.00, 0.20, 0.10]  # position of the camera on the wheelchair
CAMERA_ANGLE = np.deg2rad(15)  # camera angle around the x-axis in radians
MAX_OBJECT_HISTORY = 1  # determines how many past instances of an object are remembered, used for filtering final results
DETECTABLE_OBJECTS = [67]  # IDs of the objects that need to be detected

# replace these later:
WHEELCHAIR_POS = [0, 0, 0]
WHEELCHAIR_ANGLE = 0

def main(_argv):
    pass

# Converts pixels to meters for this specific camera
def calculate_length(pixels, distance):
    return pixels*distance*0.0014

def absolute_position(rel_pos, wheelchair_pos, angle):
    # uses a rotation matrix to calculate the position of the object relative to the wheelchair
    rel_pos = np.array([[rel_pos[0],], [rel_pos[1]], [rel_pos[2]]])
    rot_mat = np.array([[1, 0, 0], [0, np.cos(CAMERA_ANGLE), -np.sin(CAMERA_ANGLE)], [0, np.sin(CAMERA_ANGLE), np.cos(CAMERA_ANGLE)]])
    result = np.dot(rot_mat, rel_pos)
    result[0] = result[0]+CAMERA_DISPLACEMENT[0]
    result[1] = result[1]+CAMERA_DISPLACEMENT[1]
    result[2] = result[2]+CAMERA_DISPLACEMENT[2]
    # uses a rotation matrix to calculate the position of the object relative to the world
    rot_mat = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    result = np.dot(rot_mat, result)
    x = (result[0]+wheelchair_pos[0])[0]
    y = (result[1]+wheelchair_pos[1])[0]
    z = (result[2]+wheelchair_pos[2])[0]
    return x, y, z

# Determines the distance to the object and calculates the object's position
def calculate_relative_position(box, depth_frame, depth_intrin):
    x_mid = int((box[0]+box[2])/2)
    y_mid = int((box[1]+box[3])/2)
    pixel_depths = []
    for i in range(3):
        for j in range(3):
            pixel_depths.append(depth_frame.get_distance(int(x_mid+i-1),int(y_mid+j-1)))
    object_depth = statistics.median(pixel_depths)
    # If detection of object depth failed
    if object_depth == 0.0:
        print('depth not found')
        return None, None
    else:
        object_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x_mid, y_mid], object_depth)
        object_xyz = [object_point[0], object_point[2], -object_point[1]]
        #depth_colormap[max(0, min(y_mid, 479)), max(0, min(x_mid, 639))] = [0,255,0]
        return object_xyz, object_depth

# Clears all markers with IDs starting from the current amount of markers (so only unwanted markers)
def clear_markers(amount, largest_amount):
    empty_markers = MarkerArray()
    for i in range(largest_amount-amount):
        empty_marker = Marker()
        empty_marker.header.frame_id = "base_link"
        empty_marker.type = Marker.CUBE
        empty_marker.pose.position.x = 0
        empty_marker.pose.position.y = 0
        empty_marker.pose.position.z = 0
        empty_marker.scale.x = 0.1
        empty_marker.scale.y = 0.1
        empty_marker.scale.z = 0.1
        empty_marker.color.a = 0
        empty_marker.pose.orientation.w = 1.0
        empty_marker.id = i + amount
        empty_markers.markers.append(empty_marker)
    publisher.publish(empty_markers)

class Object():
    '''
    This class requires the following arguments:
    -the index of the objecttype
    -the relative position of the object to the camera as [x, y, z] in meters
    -the distance from the camera to the object in meters
    -the position of the wheelchair as [x, y, z] in meters
    -the angle of the wheelchair around the z-axis in radians
    -the bounding box of the recognized object
    '''
    def __init__(self, object_type, rel_pos, distance, wheelchair_pos, angle, bbox):
        self.recent_detections = []
        self.type = object_type
        self.update_pose(rel_pos, distance, wheelchair_pos, angle, bbox)

    def update_pose(self, rel_pos, distance, wheelchair_pos, angle, bbox):
        self.x, self.y, self.z = absolute_position(rel_pos, wheelchair_pos, angle)
        self.scale_x = calculate_length(box[2]-box[0], object_depth)
        self.scale_y = OBJECT_DEPTH_ESTIMATE
        self.scale_z = calculate_length(box[3]-box[1], object_depth)
        self.orientation_z = angle
        self.recent_detections.append([[self.x, self.y, self.z], [self.scale_x, self.scale_y, self.scale_z], self.orientation_z])
        if len(self.recent_detections) > MAX_OBJECT_HISTORY:
            self.recent_detections.pop(0)

    # Calculates new values based on previous detections.
    def average_detections(self):
        amount = len(self.recent_detections)
        total_x, total_y, total_z, total_scale_x, total_scale_y, total_scale_z, total_orientation = 0, 0, 0, 0, 0, 0, 0
        for i in range(amount):
            total_x += self.recent_detections[i][0][0]*(1/amount)
            total_y += self.recent_detections[i][0][1]*(1/amount)
            total_z += self.recent_detections[i][0][2]*(1/amount)
            total_scale_x += self.recent_detections[i][1][0]*(1/amount)
            total_scale_y += self.recent_detections[i][1][1]*(1/amount)
            total_scale_z += self.recent_detections[i][1][2]*(1/amount)
            total_orientation += self.recent_detections[i][2]*(1/amount)
        return total_x, total_y, total_z, total_scale_x, total_scale_y, total_scale_z, total_orientation

    def marker(self):
        object_marker = Marker()
        object_marker.header.frame_id = "base_link"
        object_marker.type = Marker.CUBE
        x, y, z, scale_x, scale_y, scale_z, orientation_z = self.average_detections()
        object_marker.pose.position.x = x
        object_marker.pose.position.y = y
        object_marker.pose.position.z = z
        object_marker.scale.x = scale_x
        object_marker.scale.y = scale_y
        object_marker.scale.z = scale_z
        object_marker.color.r = 0.1
        object_marker.color.g = 0.1
        object_marker.color.b = 0.1
        object_marker.color.a = 1
        object_marker.pose.orientation.z = orientation_z
        object_marker.pose.orientation.w = 1.0
        return object_marker

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

    # Creates a ROS node for communication with RViz
    publisher = rospy.Publisher("object_detection", MarkerArray, queue_size=100)
    rospy.init_node("visualization")

    # create a marker for the camera
    camera = Marker()
    camera.header.frame_id = "base_link"
    camera.type = Marker.CUBE
    camera.pose.position.x = CAMERA_DISPLACEMENT[0]
    camera.pose.position.y = CAMERA_DISPLACEMENT[1]
    camera.pose.position.z = CAMERA_DISPLACEMENT[2]
    camera.scale.x = 0.09
    camera.scale.y = 0.025
    camera.scale.z = 0.025
    camera.color.r = 0.8
    camera.color.g = 0.8
    camera.color.b = 0.8
    camera.color.a = 1
    camera.pose.orientation.x = CAMERA_ANGLE
    camera.pose.orientation.w = 1.0

    # Detect object at a given frequency
    rate = rospy.Rate(10) # hz
    object_list = []
    largest_index = 0
    while not rospy.is_shutdown():
        markers = MarkerArray()
        markers.markers.append(camera)

	# Read camera frames
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

	# Get the intrinsics of the video streams to later calculate a 3D position
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)

	# Change the frames into an image
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

	# Feed the color image into the neural network
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

	# Draw boxes around the detected objects
        bboxes = utils.postprocess_boxes(pred_bbox, frame_size, input_size, 0.25)
        bboxes = utils.nms(bboxes, 0.213, method='nms')

        # Filter objects of interest
        detected_objects = []
        for box in bboxes:
            for object_id in DETECTABLE_OBJECTS:
                if box[5] == object_id:
                    print("found", object_id)
                    detected_objects.append(box.tolist())

	# Loop over each previously detected object
        for known_object in object_list:
            closest_match = None
            closest_match_distance = None
            for box in detected_objects:
                # Check which of the detected objects of the same type lies closest
                if box[5] == known_object.type:
                    object_xyz_relative, object_depth = calculate_relative_position(box, depth_frame, depth_intrin)
                    if object_xyz_relative is not None:
                        object_x, object_y, object_z = absolute_position(object_xyz_relative, WHEELCHAIR_POS, WHEELCHAIR_ANGLE)
                        distance = abs(object_x-known_object.x)+abs(object_y-known_object.y)+abs(object_z-known_object.z)
                        if closest_match is None:
                            closest_match = box
                            closest_match_distance = distance
                        elif distance < closest_match_distance:
                            closest_match = box
                            closest_match_distance = distance
            # Update the pose of the object with the closest new detection
            if closest_match is not None:
                object_xyz_relative, object_depth = calculate_relative_position(closest_match, depth_frame, depth_intrin)
                known_object.update_pose(object_xyz_relative, object_depth, WHEELCHAIR_POS, WHEELCHAIR_ANGLE, box)
                # Remove the detection from the list as it has been matched with a previously detected object
                detected_objects.remove(closest_match)
            # What to do when an object can't find a match, possibly remove it
            else:
                # TODO
                pass
        # If there are still detections remaining after all objects have been matched, create a new object for each new detection
        for new_object_box in detected_objects:
            object_xyz_relative, object_depth = calculate_relative_position(new_object_box, depth_frame, depth_intrin)
            if object_xyz_relative is not None:
                new_object = Object(new_object_box[5], object_xyz_relative, object_depth, WHEELCHAIR_POS, WHEELCHAIR_ANGLE, new_object_box)
                object_list.append(new_object)
        print(object_list)

	# Show the detected objects
        cv2.namedWindow("result", cv2.WINDOW_AUTOSIZE)
        image_color = utils.draw_bbox(frame, bboxes)
        result = cv2.cvtColor(image_color, cv2.COLOR_RGB2BGR)
        image_depth = utils.draw_bbox(depth_colormap, bboxes)
        cv2.imshow("result", image_depth)
        print('-----')
        if cv2.waitKey(1) & 0xFF == ord('q'):
            pipeline.stop()
            break

        # Creates markers for every detected object
        for detected_object in object_list:
            markers.markers.append(detected_object.marker())

        index = 0
        for m in markers.markers:
            m.id = index
            index += 1

        # Memorizes the largest amount of objects saved at one time to be able to clear the markers
        if index > largest_index:
            largest_index = index
        clear_markers(index, largest_index)

        publisher.publish(markers)
        rate.sleep()

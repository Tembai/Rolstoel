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
import random

flags.DEFINE_string('framework', 'tf', '(tf, tflite')
flags.DEFINE_string('weights', './data/yolov4.weights',
                    'path to weights file')
flags.DEFINE_integer('size', 608, 'resize images to')
flags.DEFINE_boolean('tiny', False, 'yolo or yolo-tiny')
flags.DEFINE_string('model', 'yolov4', 'yolov3 or yolov4')

'''
Assuming the camera hasn't been rotated:
A positive x means going to the right of the camera.
A positive y means going in the direction the camera is facing.
A positive z means going upwards.
'''
CAMERA_DISPLACEMENT = [0.00, 0.00, 0.00]  # position of the camera on the wheelchair
CAMERA_ANGLE = np.deg2rad(0)  # camera angle around the x-axis in radians
CAMERA_VIEW_ANGLE_HORIZONTAL = np.deg2rad(69.4)  # the horizontal field of view of the camera in radians
RESOLUTION_WIDTH = 640
RESOLUTION_HEIGHT = 480
# IDs of the objects that need to be detected, more objects make the detection speed slower
# Possible indoor obstacles:
#DETECTABLE_OBJECTS = [0,1,2,3,13,15,16,24,25,26,28,30,31,32,33,34,35,36,37,38,39,40,41,45,56,57,58,59,60,61,62,63,68,69,70,71,72,75,77]
# All objects:
#DETECTABLE_OBJECTS = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79]
# Just phones and bottles:
DETECTABLE_OBJECTS = [39, 67]

# replace these later:
WHEELCHAIR_POS = [0, 0, 0]
WHEELCHAIR_ANGLE = np.deg2rad(0)

flip_camera_ver = False
flip_camera_hor = False

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, RESOLUTION_WIDTH, RESOLUTION_HEIGHT, rs.format.z16, 30)
config.enable_stream(rs.stream.color, RESOLUTION_WIDTH, RESOLUTION_HEIGHT, rs.format.bgr8, 30)

# Start streaming and calculate the depth scale
profile = pipeline.start(config)
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

# Prepares the alignment of the depth image to the color image
align_to = rs.stream.color
align = rs.align(align_to)


def main(_argv):
    pass

# Converts pixels to meters for this specific camera
def calculate_length(pixels, distance):
    return pixels*distance*0.0014

# Calculates the distance to the object and the object's angle
def detect_object_depth_orientation(box, depth_frame):
    # Determines how far apart the depth of points can lie before they're considered a different depth, this has no unit
    MAX_DEPTH_TOLERANCE = 20
    # Minimal size an object needs to be compared to its bounding box, in percentage, used to filter noise
    MIN_OBJECT_SIZE_PERCENTAGE = 0.02

    min_object_size = (box[2]-box[0])*(box[3]-box[1])*MIN_OBJECT_SIZE_PERCENTAGE

    # Takes the part of the image where the object was detected
    width_tolerance = 0.10*(box[2]-box[0])
    height_tolerance = 0.10*(box[3]-box[1])
    min_x = int(max(0, box[0]-width_tolerance))
    max_x = int(min(RESOLUTION_WIDTH-1, box[2]+width_tolerance))
    min_y = int(max(0, box[1]-height_tolerance))
    max_y = int(min(RESOLUTION_HEIGHT-1, box[3]+height_tolerance))
    image = np.asanyarray(depth_frame.get_data())
    if flip_camera_ver:
        image = np.flipud(image)
    if flip_camera_hor:
        image = np.fliplr(image)
    image_sliced = image[min_y:max_y, min_x:max_x]
    h, w = image_sliced.shape
    # Determines the size and location of the bounding box in the new image [x_min, y_min, x_max, y_max]
    box_relative = [int(box[0]-min_x), int(box[1]-min_y), int(w-(max_x-box[2])), int(h-(max_y-box[3]))]

    # Runs as long as the image is not completely filled with zeros
    image_sliced_copy = np.copy(image_sliced)
    layers = []
    while np.any(image_sliced_copy):
        # Filters each layer of depth points that lie around the same depth
        new_depth_layer = np.zeros(image_sliced_copy.shape)
        non_zeros = np.nonzero(image_sliced_copy)
        depth_value = image_sliced_copy[non_zeros[0][0], non_zeros[1][0]]
        max_depth, min_depth = depth_value, depth_value
        # Checks how deep this depth layer is
        while np.logical_and(image_sliced_copy > max_depth, image_sliced_copy <= max_depth+MAX_DEPTH_TOLERANCE).any():
            max_depth = max_depth+1
        while np.logical_and(image_sliced_copy < min_depth, image_sliced_copy >= min_depth-MAX_DEPTH_TOLERANCE).any():
            min_depth = min_depth-1
        # Copies all pixels of this depth to a different image and remove the copied pixels
        depth_indices = np.where(np.logical_and(image_sliced_copy >= min_depth, image_sliced_copy <= max_depth))
        if len(depth_indices[0]) >= min_object_size:
            new_depth_layer[depth_indices] = image_sliced_copy[depth_indices]
            layers.append(np.copy(new_depth_layer))
        image_sliced_copy[depth_indices] = 0
    
    # Finds the layer that is most likely to belong to the object
    object_layer = None
    object_layer_score = 0
    for layer in layers:
        # Determines how much the depth of the current layer fills the bounding box
        image_box = layer[box_relative[1]:box_relative[3], box_relative[0]:box_relative[2]]
        h, w = image_box.shape
        pixel_amount_inside = cv2.countNonZero(image_box)
        percentage_filled = pixel_amount_inside/(h*w)
        # Determines how much the depth of the current layer is outside the bounding box
        pixel_amount_total = cv2.countNonZero(layer)
        pixel_amount_outside = max(1, pixel_amount_total-pixel_amount_inside)  # Makes sure there's no division by 0 later on
        percentage_outside = pixel_amount_outside/pixel_amount_total
        # Calculates a score based on how much the bounding box is filled and how much of the layer lies outside of it
        # Score calculation could possibly be improved
        score = percentage_filled-percentage_outside
        # Remembers the best scored layer
        if object_layer is not None:
            if score > object_layer_score:
                object_layer = image_box
                object_layer_score = score
        else:
            object_layer = image_box
            object_layer_score = score

    # If something went wrong
    if object_layer is None:
        return None, None
    
    # Calculates the average depth of the object
    average_pixel_value = object_layer[object_layer!=0].mean()
    average_pixel_depth = average_pixel_value*depth_scale

    # Calculates the average distance to the left side of the object
    h, w = object_layer.shape
    half_width = int(w/2)
    object_left_side = object_layer[:, :half_width]
    average_pixel_value_left = object_left_side[object_left_side!=0].mean()
    average_pixel_depth_left = average_pixel_value_left*depth_scale
    # Calculates the average distance to the right side of the object
    object_right_side = object_layer[:, half_width:]
    average_pixel_value_right = object_right_side[object_right_side!=0].mean()
    average_pixel_depth_right = average_pixel_value_right*depth_scale
    # Calculates the angle at which the object is positioned
    object_length = calculate_length(w, average_pixel_depth)
    depth_difference = average_pixel_depth_right-average_pixel_depth_left
    angle = np.tan(depth_difference/object_length)
    return average_pixel_depth, angle

def absolute_position(rel_pos, wheelchair_pos, wheelchair_angle):
    # rel_pos is the position relative to the camera
    # uses a rotation matrix to calculate the position of the object relative to the wheelchair
    rel_pos = np.array([[rel_pos[0],], [rel_pos[1]], [rel_pos[2]]])
    rot_mat = np.array([[1, 0, 0], [0, np.cos(CAMERA_ANGLE), -np.sin(CAMERA_ANGLE)], [0, np.sin(CAMERA_ANGLE), np.cos(CAMERA_ANGLE)]])
    result = np.dot(rot_mat, rel_pos)
    result[0] = result[0]+CAMERA_DISPLACEMENT[0]
    result[1] = result[1]+CAMERA_DISPLACEMENT[1]
    result[2] = result[2]+CAMERA_DISPLACEMENT[2]
    # uses a rotation matrix to calculate the position of the object relative to the world
    rot_mat = np.array([[np.cos(wheelchair_angle), -np.sin(wheelchair_angle), 0], [np.sin(wheelchair_angle), np.cos(wheelchair_angle), 0], [0, 0, 1]])
    result = np.dot(rot_mat, result)
    x = (result[0]+wheelchair_pos[0])[0]
    y = (result[1]+wheelchair_pos[1])[0]
    z = (result[2]+wheelchair_pos[2])[0]
    return x, y, z

# Determines the distance to the object and calculates the object's pose
def calculate_relative_pose(box, depth_frame, depth_intrin):
    x_mid = int((box[0]+box[2])/2)
    y_mid = int((box[1]+box[3])/2)
    object_depth, object_angle = detect_object_depth_orientation(box, depth_frame)
    if object_depth is not None:
        # The camera intrinsics couldn't be flipped so the other data has to be temporarily flipped
        if flip_camera_ver:
            y_mid = RESOLUTION_HEIGHT-y_mid
        if flip_camera_hor:
            x_mid = RESOLUTION_WIDTH-x_mid
        object_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x_mid, y_mid], object_depth)
        if flip_camera_ver:
            object_point[1] = -object_point[1]
        if flip_camera_hor:
            object_point[0] = -object_point[0]
        object_xyz = [object_point[0], object_point[2], -object_point[1]]
        return object_xyz, object_depth, object_angle
    else:
        return None, None, None

# Calculates if a given point is in view of the camera in 2D
def point_in_view(point_x, point_y, camera_x, camera_y, wheelchair_angle, max_distance):
    # Calculates the position of the point relative to the camera
    point_x = point_x-camera_x
    point_y = point_y-camera_y
    position = np.array([[point_x], [point_y]])
    rot_mat = np.array([[np.cos(-WHEELCHAIR_ANGLE), -np.sin(-WHEELCHAIR_ANGLE)], [np.sin(-WHEELCHAIR_ANGLE), np.cos(-WHEELCHAIR_ANGLE)]])
    result = np.dot(rot_mat, position)
    # Checks if the point is behind the camera or too far away
    distance = result[1][0]
    if distance <= 0 or distance > max_distance:
        return False
    else:
        # Checks if the point lies within the triangle-shaped view of the camera
        x_position = result[0][0]
        outer_x = distance*np.sin(CAMERA_VIEW_ANGLE_HORIZONTAL)
        if x_position > -outer_x and x_position < outer_x:
            return True
        else:
            return False

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

# Determines how far the camera can see in a given instance
def max_view_distance(frame):
    image = np.asanyarray(frame.get_data())
    if flip_camera_ver:
        image = np.flipud(image)
    if flip_camera_hor:
        image = np.fliplr(image)
    image_scaled = cv2.convertScaleAbs(image, alpha=0.08)
    # Filter all values of 255 which are usually reflections
    image_scaled[image_scaled == 255] = 0
    # Find the position of the pixel that is furthest away from the camera
    max_value = np.amax(image_scaled)
    furthest_pixels = np.where(image_scaled == max_value)
    if flip_camera_ver:
        furthest_pixels[1][0] = RESOLUTION_WIDTH-furthest_pixels[1][0]-1
    if flip_camera_hor:
        furthest_pixels[0][0] = RESOLUTION_HEIGHT-furthest_pixels[0][0]-1
    furthest_depth = frame.get_distance(furthest_pixels[1][0], furthest_pixels[0][0])
    return furthest_depth

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
    def __init__(self, object_type, rel_pos, rel_angle, distance, wheelchair_pos, wheelchair_angle, bbox):
        self.type = object_type
        self.update_pose(rel_pos, rel_angle, distance, wheelchair_pos, wheelchair_angle, bbox)

    def update_pose(self, rel_pos, rel_angle, distance, wheelchair_pos, wheelchair_angle, bbox):
        self.x, self.y, self.z = absolute_position(rel_pos, wheelchair_pos, wheelchair_angle)
        self.scale_x = calculate_length(bbox[2]-bbox[0], object_depth)
        self.scale_z = calculate_length(bbox[3]-bbox[1], object_depth)
        # Uses the average of the object's height and length to estimate it's depth
        self.scale_y = (self.scale_x+self.scale_z)/2
        self.orientation_z = rel_angle+wheelchair_angle

    def marker(self):
        object_marker = Marker()
        object_marker.header.frame_id = "base_link"
        object_marker.type = Marker.CUBE
        object_marker.pose.position.x = self.x
        object_marker.pose.position.y = self.y
        object_marker.pose.position.z = self.z
        object_marker.scale.x = self.scale_x
        object_marker.scale.y = self.scale_y
        object_marker.scale.z = self.scale_z
        # Picks a random color for each type of object
        random.seed(self.type)
        object_marker.color.r = random.uniform(0.0, 1.0)
        object_marker.color.g = random.uniform(0.0, 1.0)
        object_marker.color.b = random.uniform(0.0, 1.0)
        object_marker.color.a = 1
        object_marker.pose.orientation.z = self.orientation_z
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
        if flip_camera_ver:
            depth_image = np.flipud(depth_image)
            color_image = np.flipud(color_image)
        if flip_camera_hor:
            depth_image = np.fliplr(depth_image)
            color_image = np.fliplr(color_image)

        frame = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(frame)

        frame_size = frame.shape[:2]
        image_data = utils.image_preprocess(np.copy(frame), [input_size, input_size])
        image_data = image_data[np.newaxis, ...].astype(np.float32)

        scaled_depth = cv2.convertScaleAbs(depth_image, alpha=0.08)
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

        # Filter previously detected objects that are in view
        visible_object_list = []
        camera_x, camera_y, camera_z = absolute_position([0, 0, 0], WHEELCHAIR_POS, WHEELCHAIR_ANGLE)
        view_distance = max_view_distance(depth_frame)
        for known_object in object_list:
            if point_in_view(known_object.x, known_object.y, camera_x, camera_y, WHEELCHAIR_ANGLE, view_distance):
                visible_object_list.append(known_object)

	    # Loop over each previously detected object that should be in view
        for known_object in visible_object_list:
            closest_match = None
            closest_match_distance = None
            for box in detected_objects:
                # Check which of the detected objects of the same type lies closest
                if box[5] == known_object.type:
                    object_xyz_relative, object_depth, object_angle = calculate_relative_pose(box, depth_frame, depth_intrin)
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
                object_xyz_relative, object_depth, object_angle = calculate_relative_pose(closest_match, depth_frame, depth_intrin)
                if object_xyz_relative is not None:
                    known_object.update_pose(object_xyz_relative, object_angle, object_depth, WHEELCHAIR_POS, WHEELCHAIR_ANGLE, closest_match)
                    # Remove the detection from the list as it has been matched with a previously detected object
                    detected_objects.remove(closest_match)
            # When an object can't find a match, remove it
            else:
                object_list.remove(known_object)
        # If there are still detections remaining after all objects have been matched, create a new object for each new detection
        for new_object_box in detected_objects:
            object_xyz_relative, object_depth, object_angle = calculate_relative_pose(new_object_box, depth_frame, depth_intrin)
            if object_xyz_relative is not None:
                new_object = Object(new_object_box[5], object_xyz_relative, object_angle, object_depth, WHEELCHAIR_POS, WHEELCHAIR_ANGLE, new_object_box)
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


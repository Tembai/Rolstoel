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

def main(_argv):
    import os
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

    while True:
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


        view2d = np.zeros((480,640,3),np.uint8)

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
                depth_colormap[max(0, min(y_mid, 479)), max(0, min(x_mid, 639))] = [0,255,0]
                view2d[max(0, min(480-int(object_point[2]*350), 479)), max(0, min(int(object_point[0]*350)+320, 639))] = [0,255,0]
            #print('x_min', box[0])
            #print('y_min', box[1])
            #print('x_max', box[2])
            #print('y_max', box[3])
            #print('probability', box[4])
            #print('object_id', box[5])
            #print('point', object_point)
            #print('-----')

        #curr_time = time.time()
        #exec_time = curr_time - prev_time
        #info = "time: %.2f ms" %(1000*exec_time)
        #print(info)
        cv2.namedWindow("result", cv2.WINDOW_AUTOSIZE)
        image_color = utils.draw_bbox(frame, bboxes)
        result = cv2.cvtColor(image_color, cv2.COLOR_RGB2BGR)
        image_depth = utils.draw_bbox(depth_colormap, bboxes)
        images = np.hstack((view2d, image_depth))
        cv2.imshow("result", images)
        print('-----')
        if cv2.waitKey(1) & 0xFF == ord('q'):
            pipeline.stop()
            break

if __name__ == '__main__':
    try:
        app.run(main)
    except SystemExit:
        pass

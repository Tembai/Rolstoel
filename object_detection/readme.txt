Note: if you encounter problems while installing the object detection program try using pip3 and python3 when entering commands instead of pip and python.

FIRST TIME SET-UP:
1:
Download the following repository as a zip-file and unpack: https://github.com/hunglc007/tensorflow-yolov4-tflite
Download the following file: https://drive.google.com/file/d/1cewMfusmPjYWbrnuJRuKhPMwRe_b9PaT/view

2:
In the tensorflow folder, run the following command to use your gpu: pip install -r requirements-gpu.txt
Or to use your cpu run: pip install -r requirements.txt

3:
Move the downloaded yolov4.weights file to the tensorflow/data folder.

4:
Copy the files from the core_replace folder (these are config.py, utils.py and yolov4.py) to the tensorflow/core folder and overwrite them

DETECTING OBJECTS:
1:
Open a new terminal and run: roscore
Open another terminal and run: rosrun rviz rviz
Open yet another new terminal and run: rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map base_link 10
Move the object detection python file into the tensorflow folder and open a terminal at the location of the tensorflow folder and run the python file.

2:
After the python file has succesfully launched click on the Add button on the bottom left in RViz.
Then under "by topic" double click MarkerArray under the /object_detection topic.

If everything works correctly you should see markers being placed in RViz.

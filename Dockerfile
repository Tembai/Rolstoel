
## -*- docker-image-name: "my-ros-gazebo" -*-

FROM ubuntu:18.04
LABEL MAINTAINER="Rufus Fraanje"

ARG user_id=1000
ARG group_id=1000
ARG user_name=user
ARG group_name=user

# Build docker file with
# docker build -t my-env-a .
# Run docker file with
# docker run --rm -e DISPLAY=$DISPLAY  -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/:/mnt/work my-env-a

ENV DEBIAN_FRONTEND noninteractive

# no caching of Python bytecode
ENV PYTHONDONTWRITEBYTECODE=true
ENV PYTHONUNBUFFERED=1

RUN groupadd --gid ${group_id} ${group_name}\
    && useradd --uid ${user_id} --gid ${group_name} \
       --create-home --home-dir /home/${user_name} \
       --groups sudo \
       --password "" \
       --shell /bin/bash ${user_name} 

ENV HOME /home/${user_name}

RUN echo 'APT::Get::Assume-Yes "true";' >> /etc/apt/apt.conf \
    && apt-get update \
    && apt-get install apt-utils \
    && apt-get install -y locales locales-all \
    && sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen \
    && dpkg-reconfigure --frontend=noninteractive locales \
    && update-locale LANG=en_US.UTF-8 \
    && apt-get install sudo

ENV LANG en_US.UTF-8  
ENV LC_ALL en_US.UTF-8  


RUN apt-get install wget curl module-init-tools

RUN apt-get install python3-pip python3-dev 

RUN pip3 install --upgrade pip \
    && pip3 install virtualenv \
    && pip3 install numpy \
    && pip3 install scipy \
    && pip3 install matplotlib \
    && pip3 install PyQt5 \
    && pip3 install PyOpenGL \
    && pip3 install PyOpenGL_accelerate \
    && pip3 install pyqtgraph \
    && pip3 install jupyter

#https://stackoverflow.com/questions/44166269/libgl-error-failed-to-load-driver-swrast-in-docker-container
#add nvidia driver to docker file:
#COPY /home/rufus/Downloads/NVIDIA-Linux-x86_64-440.100.run .

RUN apt-get purge nvidia* \
    && apt-get install kmod \
    && wget http://us.download.nvidia.com/XFree86/Linux-x86_64/440.100/NVIDIA-Linux-x86_64-440.100.run \
    && export TERM=linux \
    && chmod a+x NVIDIA-Linux-x86_64-440.100.run \
    && ./NVIDIA-Linux-x86_64-440.100.run --silent \
                                        --no-kernel-module \
                                        --install-compat32-libs \
                                        --no-nouveau-check \
                                        --no-nvidia-modprobe \
                                        --no-rpms \
                                        --no-backup \
                                        --no-check-for-alternate-installs \
                                        --no-libglx-indirect \
                                        --no-install-libglvnd \
                                        --x-prefix=/tmp/null \
                                        --x-module-path=/tmp/null \
                                        --x-library-path=/tmp/null \
                                        --x-sysconfig-path=/tmp/null

RUN apt-get update --fix-missing && apt-get install libgl1-mesa-glx libgl1-mesa-dri mesa-utils 
RUN apt-get install freeglut3 freeglut3-dev libxi-dev libxmu-dev

ENV DISTRIB_CODENAME=bionic

RUN sh -c '. /etc/lsb-release && echo "deb http://ftp.tudelft.nl/ros/ubuntu $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update \
    && apt-get install ros-melodic-desktop-full \
    && apt-get install python-rosdep

RUN apt-get update --fix-missing \
    && apt-get install tmux git vim

RUN apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy \
                    ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch \
                    ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino \
                    ros-melodic-rosserial-python ros-melodic-rosserial-server \
                    ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl \
                    ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
                    ros-melodic-compressed-image-transport ros-melodic-rqt-image-view \
                    ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers

RUN apt-get install ros-melodic-turtlebot3 ros-melodic-turtlebot3-applications \
                    ros-melodic-turtlebot3-simulations \
                    ros-melodic-open-manipulator-with-tb3-gazebo \
                    ros-melodic-open-manipulator-with-tb3-simulations \
                    ros-melodic-open-manipulator-with-tb3-tools \
                    ros-melodic-toposens-pointcloud

RUN apt-get install ros-melodic-moveit-fake-controller-manager \
                    ros-melodic-joint-trajectory-controller \
                    ros-melodic-moveit-core \
                    ros-melodic-moveit-planners-ompl \
                    ros-melodic-moveit-ros-move-group \
                    ros-melodic-moveit-ros-planning \
                    ros-melodic-moveit-simple-controller-manager \
                    ros-melodic-moveit-kinematics \
                    ros-melodic-moveit-ros-visualization \
                    ros-melodic-effort-controllers

RUN apt-get install libeigen3-dev ros-melodic-xpp

RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-repo-ubuntu1804_10.2.89-1_amd64.deb \
   && dpkg -i cuda-repo-ubuntu1804_10.2.89-1_amd64.deb \
   && rm cuda-repo-ubuntu1804_10.2.89-1_amd64.deb \
   && apt-key adv --fetch-keys 'https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub' \
   && apt-get update \
   && apt-get install cuda-toolkit-10-2

RUN rm /etc/ld.so.conf.d/cuda-10-2.conf \
    && echo "/usr/local/cuda/lib64" > /etc/ld.so.conf.d/cuda.conf \
    && ldconfig \
    && ln -s /usr/local/cuda /usr/local/cuda-10.2

COPY cuda.sh /etc/profile.d/cuda.sh
 
RUN apt-get install build-essential cmake libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libgtk-3-dev libatlas-base-dev liblapack-dev libblas-dev gfortran python3-dev pkg-config unzip ffmpeg qtbase5-dev  python3-dev python3-numpy libhdf5-dev libgtk-3-dev libdc1394-22 libdc1394-22-dev libjpeg-dev libtiff5-dev libtesseract-dev libavcodec-dev libavformat-dev libswscale-dev libxine2-dev libgstreamer-plugins-base1.0-0 libgstreamer-plugins-base1.0-dev libpng16-16 libpng-dev libv4l-dev libtbb-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev v4l-utils libleptonica-dev

ENV PATH=/usr/local/cuda/bin:$PATH
ENV D_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64:/usr/local/lib:$LD_LIBRARY_PATH
ENV CUDADIR=/usr/local/cuda
ENV CUDA_HOME=/usr/local/cuda
ENV CUDA_ROOT=/usr/local/cuda/bin

RUN pip3 install pycuda

RUN pip3 install torch==1.5.0+cu101 torchvision==0.6.0+cu101 -f https://download.pytorch.org/whl/torch_stable.html \
   && pip3 install tensorboard

RUN apt-get update --fix-missing \
     && apt-get install ros-melodic-husky-navigation \
                        ros-melodic-husky-desktop \
                        ros-melodic-husky-gazebo \
                        ros-melodic-cartographer-ros

RUN apt-get install ros-melodic-velodyne-pointcloud \
                    ros-melodic-velocity-controllers \
                    ros-melodic-joint-state-publisher-gui

RUN apt-get install ros-melodic-rosbridge-suite
RUN apt-get install ros-melodic-rospy-message-converter python3-yaml \
    && pip3 install rospkg catkin_pkg \
    && pip3 install flatten_json

RUN apt-get install python-dev python-opencv \
        python-pip \
	libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler



RUN apt-get install --no-install-recommends libboost-all-dev && \
	apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev && \
	pip2 install protobuf && \
	mkdir -p /opt && \
        cd /opt/ && \
	git clone https://github.com/BVLC/caffe.git && \
	cd caffe && \
	cp Makefile.config.example Makefile.config && \
	echo "OPENCV_VERSION := 3" >> Makefile.config && \
        echo "BLAS := atlas" >> Makefile.config && \
	echo "CUDA_ARCH := -gencode arch=compute_61,code=sm_61" >> Makefile.config && \	
        echo "CUDA_DIR := /usr/local/cuda" >> Makefile.config && \	  
	echo "PYTHON_LIBRARIES := boost_python python2.7"  >> Makefile.config && \
	echo "PYTHON_INCLUDE := /usr/include/python2.7 /usr/lib/python2.7/dist-packages/numpy/core/include"  >> Makefile.config && \
	echo "INCLUDE_DIRS := /usr/include/python2.7 /usr/lib/python2.7/dist-packages/numpy/core/include /usr/local/include /usr/include/hdf5/serial/" >> Makefile.config && \
        echo "LIBRARY_DIRS := $(PYTHON_LIB) /usr/local/lib /usr/lib/x86_64-linux-gnu/hdf5/serial"  >> Makefile.config && \
	ln -s /usr/lib/x86_64-linux-gnu/libhdf5_serial.so /usr/lib/x86_64-linux-gnu/libhdf5.so && \
	ln -s /usr/lib/x86_64-linux-gnu/libhdf5_serial_hl.so /usr/lib/x86_64-linux-gnu/libhdf5_hl.so && \
 	make all -j4 && \
	make pycaffe -j4
	

RUN apt-get update && apt-get install ros-melodic-moveit-setup-assistant \
	ros-melodic-capabilities \
	ros-melodic-yocs-cmd-vel-mux \
	ros-melodic-octomap-ros \
	ros-melodic-yocs-velocity-smoother

# apparently necessary for caffe:
RUN apt-get install python-sklearn python-skimage

# clean up
#RUN apt-get autoremove \
#    && rm -rf /tmp/* /var/lib/apt/lists/* /root/.cache/*

COPY bashrc /home/user/.bashrc
COPY start.sh /usr/local/bin/

EXPOSE 80
EXPOSE 8888
EXPOSE 8080
EXPOSE 5000

USER ${user_name}
WORKDIR /home/${user_name}

RUN sudo rosdep init && rosdep update

#ENTRYPOINT ["/usr/local/bin/start.sh" ]
CMD ["bash", "-i"]


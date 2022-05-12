#FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu20.04
FROM nvidia/cudagl:11.4.0-base-ubuntu20.04

# Install packages without prompting the user to answer any questions
ENV DEBIAN_FRONTEND=noninteractive


#####################################################
# Install common apt packages
#####################################################
# RUN rm /etc/apt/sources.list.d/cuda.list
# RUN rm /etc/apt/sources.list.d/nvidia-ml.list
RUN apt-key del 7fa2af80
RUN apt-get update && apt-get install -y --no-install-recommends wget
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-keyring_1.0-1_all.deb
RUN dpkg -i cuda-keyring_1.0-1_all.deb

RUN apt-get update && apt-get install -y \
	### utility
	locales \
	xterm \
	dbus-x11 \
	terminator \
	sudo \
	### tools
	unzip \
	lsb-release \
	curl \
	ffmpeg \
	net-tools \
	software-properties-common \
	subversion \
	libssl-dev \
	### Development tools
	build-essential \
	htop \
	git \
	vim \
	gedit \
	gdb \
	valgrind \
	&& apt-get clean && rm -rf /var/lib/apt/lists/*


#####################################################
# Set locale & time zone
#####################################################
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8
ENV TZ=Asia/Tokyo


#####################################################
# cmake 3.15.5
#####################################################
RUN git clone https://gitlab.kitware.com/cmake/cmake.git && \
	cd cmake && \
	git checkout tags/v3.16.3 && \
	./bootstrap --parallel=8 && \
	make -j8 && \
	make install && \
	cd .. && rm -rf cmake


#####################################################
# Python 3.8
#####################################################
RUN add-apt-repository ppa:deadsnakes/ppa
RUN apt-get update && apt-get install -y \
  python3.8 \
  python3-pip \
	&& apt-get clean && rm -rf /var/lib/apt/lists/*
RUN echo "alias python='python3'" >> /root/.bashrc


#####################################################
# Install common pip packages
#####################################################
COPY pip/requirements.txt requirements.txt
RUN pip install -r requirements.txt


#####################################################
# ROS
#####################################################
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get install -y \
	libfcl* \
	libglew-dev \
	ros-noetic-desktop-full \
	ros-noetic-joy \
	ros-noetic-gazebo* \
	ros-noetic-moveit* \
	ros-noetic-image-view* \
	ros-noetic-cv-camera* \
	ros-noetic-joint* \
	ros-noetic-graph* \
	ros-noetic-ros-controller* \
	ros-noetic-joy-teleop* \
	ros-noetic-eigen* \
	ros-noetic-rosbridge-server* \
	ros-noetic-geometric* \
	ros-noetic-object-recognition* \
	ros-noetic-map-server* \
	ros-noetic-warehouse-ros* \
	ros-noetic-geodesy && \
	apt-get clean && rm -rf /var/lib/apt/lists/*

RUN echo "export ROSLAUNCH_SSH_UNKNOWN=1" >> /root/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "export ROS_HOSTNAME=localhost" >> /root/.bashrc
RUN echo "export ROS_IP=localhost" >> /root/.bashrc
RUN echo "alias cm='cd /home/$USER/catkin_ws && catkin_make'" >> /root/.bashrc
RUN echo "alias sourceros='source ~/catkin_ws/devel/setup.bash'" >> /root/.bashrc


#####################################################
# Pytorch
#####################################################
COPY pip/requirements_pytorch.txt requirements_pytorch.txt
RUN pip install -r requirements_pytorch.txt


#####################################################
# MuJoCo 200
#####################################################
COPY packages/.mujoco /root/.mujoco
ENV LD_LIBRARY_PATH /root/.mujoco/mujoco200/bin:${LD_LIBRARY_PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

RUN apt-get update && apt-get install -y \
	mesa-utils \
  libgl1-mesa-dev \
  libgl1-mesa-glx \
  libosmesa6-dev \
  libglew-dev \
  virtualenv \
  xpra \
  xserver-xorg-dev \
	swig \
	&& apt-get clean && rm -rf /var/lib/apt/lists/*

RUN curl -o /usr/local/bin/patchelf https://s3-us-west-2.amazonaws.com/openai-sci-artifacts/manual-builds/patchelf_0.9_amd64.elf \
    && chmod +x /usr/local/bin/patchelf


#####################################################
# OpenAI gym
#####################################################
RUN sudo git clone https://github.com/openai/gym.git && \
	cd gym && \
	sudo python3 setup.py install 
COPY pip/requirements_gym.txt requirements_gym.txt
RUN pip install -r requirements_gym.txt


#####################################################
# mujoco-py
#####################################################
RUN pip install 'mujoco-py<2.1,>=2.0'


#####################################################
# Isaac Gym
#####################################################
# COPY packages/isaacgym /root/isaacgym
# RUN cd /root/isaacgym/python && \
# 	pip install -e .

# RUN apt-get update && apt-get install -y \
# 	nvidia-driver-470 \
# 	nvidia-settings \
# 	vulkan-utils \
# 	&& apt-get clean && rm -rf /var/lib/apt/lists/*

ENV LD_PRELOAD=$LD_PRELOAD:"/usr/lib/x86_64-linux-gnu/libGLEW.so"


#####################################################
# RL
#####################################################
COPY pip/requirements_rl.txt requirements_rl.txt
RUN pip install -r requirements_rl.txt


#####################################################
# Run scripts (commands)
#####################################################

### terminator window settings
COPY assets/config /

### user group settings
COPY assets/entrypoint_setup.sh /
ENTRYPOINT ["/entrypoint_setup.sh"] /

# Run terminator
CMD ["terminator"]

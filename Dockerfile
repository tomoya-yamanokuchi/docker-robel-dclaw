# FROM nvidia/cudagl:11.4.0-base-ubuntu20.04
FROM nvidia/cudagl:11.3.1-base-ubuntu20.04

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
# Python 3.8
#####################################################
RUN add-apt-repository ppa:deadsnakes/ppa
RUN apt-get update && apt-get install -y \
  python3.8 \
  python3-pip \
	&& apt-get clean && rm -rf /var/lib/apt/lists/*
RUN pip install --upgrade requests
RUN echo "alias python='python3'" >> /root/.bashrc



#####################################################
# 					rospy
#####################################################
# ref) https://qiita.com/otamasan/items/7ac7732a5c3d47ec3028
RUN pip install --extra-index-url https://rospypi.github.io/simple rospy
RUN pip install --extra-index-url https://rospypi.github.io/simple ros-numpy


# ####################################################
# Install common pip packages
# ####################################################
COPY pip/requirements.txt requirements.txt
RUN pip install -r requirements.txt


# ####################################################
# hydra
# ####################################################
RUN pip install hydra-core==1.2.0
RUN pip install transforms3d==0.3.1


#####################################################
# Pytorch Lightning
#####################################################
RUN apt-get update
RUN pip install tensorboard==2.9.1
RUN pip install pytorch-lightning==1.6.5
RUN pip install torch==1.12.0+cu113 torchvision==0.13.0+cu113 torchaudio==0.12.0 --extra-index-url https://download.pytorch.org/whl/cu113


#####################################################
# MuJoCo 200
#####################################################
COPY packages/.mujoco /root/.mujoco
ENV LD_LIBRARY_PATH /root/.mujoco/mujoco200/bin:${LD_LIBRARY_PATH}
ENV LD_LIBRARY_PATH /home/tomoya-y/.mujoco/mujoco200/bin:${LD_LIBRARY_PATH}
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
# mujoco-py
#####################################################
# RUN pip install 'mujoco-py<2.1,>=2.0'
RUN pip install mujoco-py==2.0.2.13
ENV LD_PRELOAD=$LD_PRELOAD:"/usr/lib/x86_64-linux-gnu/libGLEW.so"


#####################################################
# Run scripts (commands)
#####################################################

### terminator window settings
COPY assets/config /

### user group settings
COPY assets/entrypoint_setup.sh /
ENTRYPOINT ["/entrypoint_setup.sh"] /

# Run terminator
# CMD ["terminator"]

# WORKDIR /home/tomoya-y/workspace/robel-dclaw-env

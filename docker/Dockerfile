
# General guidelines and recommendations for writing Dockerfile:
# https://docs.docker.com/engine/userguide/eng-image/dockerfile_best-practices/

################################################################################

# Install Ubuntu Xenial Xerus (16.04) with CUDA and OpenGL support.
# https://hub.docker.com/r/nvidia/cudagl/
# https://gitlab.com/nvidia/cudagl
# https://github.com/NVIDIA/libglvnd

# Pull official parent image from Nvidia repository.
FROM nvidia/cudagl:9.0-devel-ubuntu16.04

# Install Nvidia cuDNN library missing from parent image.
# https://gitlab.com/nvidia/cuda/blob/ubuntu16.04/9.0/devel/cudnn7/Dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
	libcudnn7=7.1.3.16-1+cuda9.0 \
	libcudnn7-dev=7.1.3.16-1+cuda9.0 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install ROS Kinetic Kame.
# http://wiki.ros.org/kinetic/Installation/Ubuntu

# Update package list.
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

# Add package keys.
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Install ROS-desktop-full packages (including ROS, rqt, rviz, and more).
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-kinetic-desktop-full \
	&& rm -rf /var/lib/apt/lists/*

# Initialize 'rosdep'.
# http://wiki.ros.org/rosdep
RUN rosdep init

################################################################################

# Install MoveIt!, because it seems to not be included in desktop-full.
RUN apt-get update && apt-get install ros-kinetic-moveit -y

################################################################################

# Install required packages/tools/dependencies for subsequent installations.
RUN apt-get update && apt-get install -y --no-install-recommends \
	apt-transport-https \
	wget \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install UR package.

RUN apt-get update && apt-get install -y \
	ros-kinetic-universal-robot


################################################################################

# Install Caffe with GPU support.
# http://caffe.berkeleyvision.org/installation.html
# https://github.com/BVLC/caffe/tree/master/docker

# Gather dependencies.
RUN apt-get update && apt-get install -y --no-install-recommends \
	build-essential \
	cmake \
	git \
	wget \
	libatlas-base-dev \
	libboost-all-dev \
	libgflags-dev \
	libgoogle-glog-dev \
	libhdf5-serial-dev \
	libleveldb-dev \
	liblmdb-dev \
	libopencv-dev \
	libprotobuf-dev \
	libsnappy-dev \
	protobuf-compiler \
	python-dev \
	python-numpy \
	python-pip \
	python-setuptools \
	python-scipy \
	&& rm -rf /var/lib/apt/lists/*

# Define and move to Caffe working directory.
ENV CAFFE_ROOT=/opt/caffe/
WORKDIR $CAFFE_ROOT

# Download sources from repository and build with GPU support.
RUN git clone -b master --depth 1 https://github.com/BVLC/caffe.git . \
	&& pip install --upgrade pip==9.0.3 \
	&& cd python && for req in $(cat requirements.txt) pydot; do pip install $req; done && cd .. \
	&& git clone https://github.com/NVIDIA/nccl.git && cd nccl && make -j install && cd .. && rm -rf nccl \
	&& mkdir build && cd build \
	&& cmake -DUSE_CUDNN=1 -DUSE_NCCL=1 .. \
	&& make -j"$(nproc)"

# Install binaries and setup paths.
ENV PYCAFFE_ROOT $CAFFE_ROOT/python
ENV PYTHONPATH $PYCAFFE_ROOT:$PYTHONPATH
ENV PATH $CAFFE_ROOT/build/tools:$PYCAFFE_ROOT:$PATH
RUN echo "$CAFFE_ROOT/build/lib" >> /etc/ld.so.conf.d/caffe.conf && ldconfig

# Return to default working directory.
WORKDIR /root/

################################################################################

# Install dependencies for 'flexbe_app' ROS package.
# https://github.com/FlexBE/flexbe_app

# Gather dependencies from Ubuntu repositories.
RUN apt-get update && apt-get install -y --no-install-recommends \
  libcanberra-gtk3-module \
	libgconf2-dev \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install Microsoft VS Code.
# https://code.visualstudio.com/docs/setup/linux#_installation

# Install the Microsoft repository key.
RUN curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg \
	&& mv microsoft.gpg /etc/apt/trusted.gpg.d/microsoft.gpg \
	&& echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list

# Update cache and install VS Code as a package.
RUN apt-get update && apt-get install -y --no-install-recommends \
	code \
	&& rm -rf /var/lib/apt/lists/*

# Set VS Code as the default text editor.
# https://code.visualstudio.com/docs/setup/linux#_setting-vs-code-as-the-default-text-editor
RUN update-alternatives --set editor /usr/bin/code

# Solve common error while deleting files from VS Code.
# https://code.visualstudio.com/docs/setup/linux#_debian-and-moving-files-to-trash
RUN apt-get update && apt-get install -y --no-install-recommends \
	gvfs-bin \
	&& rm -rf /var/lib/apt/lists/*

# Solve character rendering issues in GUI by adding missing Japanese fonts.
RUN apt-get update && apt-get install -y --no-install-recommends \
	fonts-takao \
	&& rm -rf /var/lib/apt/lists/*

# Install a dependency that the next command requires, but VS Code seems to miss sometimes
RUN apt-get update && apt-get install libxtst6

# Install additional recommended VS Code extensions.
# https://code.visualstudio.com/docs/editor/extension-gallery#_command-line-extension-management
RUN code --install-extension ms-python.python --user-data-dir=/root/.vscode/ \
	&& code --install-extension ms-vscode.cpptools --user-data-dir=/root/.vscode/ \
	&& code --install-extension PeterJausovec.vscode-docker --user-data-dir=/root/.vscode/ \
	&& code --install-extension streetsidesoftware.code-spell-checker --user-data-dir=/root/.vscode/ \
	&& code --install-extension eamodio.gitlens --user-data-dir=/root/.vscode/ \
	&& code --install-extension ajshort.ros --user-data-dir=/root/.vscode/ \
	&& code --install-extension ajshort.msg --user-data-dir=/root/.vscode/

# Install Python linter as recommended by Python extension.
RUN /usr/bin/python -m pip install -U \
	pylint \
	autopep8

# Install Exuberant Ctags as recommended by Python extension.
RUN apt-get update && apt-get install -y --no-install-recommends \
	exuberant-ctags \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install the SSH deploy key of the repository hosted on GitLab.com.

# Copy the SSH deploy key from the sources on the host to the container.
COPY ./.ssh/. /root/.ssh/
RUN chmod 700 /root/.ssh/ \
	&& chmod 600 /root/.ssh/o2as-id_rsa

# Add GitLab.com as a known host using the SSH deploy key.
RUN echo "Host gitlab.com" > /root/.ssh/config \
	&& echo "  Hostname gitlab.com" >> /root/.ssh/config \
	&& echo "  IdentityFile /root/.ssh/o2as-id_rsa" >> /root/.ssh/config \
	&& echo "  StrictHostKeyChecking no" >> /root/.ssh/config \
	&& echo "  User git" >> /root/.ssh/config

################################################################################

# Install common optional packages/tools.
RUN apt-get update && apt-get install -y --no-install-recommends \
	inetutils-ping \
	iproute2 \
	gdb \
	git \
	mesa-utils \
	nano \
	net-tools \
	openssh-client \
	python-catkin-tools \
	python-sklearn \
	python-tk \
	libusb-dev \
	&& rm -rf /var/lib/apt/lists/*

# TODO: Remove libusb-dev and leave it in as a o2as_usb_relay dependency.

################################################################################

# Install camera packages.

# librealsense2
RUN rm -f /etc/apt/sources.list.d/realsense-public.list
RUN echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | tee /etc/apt/sources.list.d/realsense-public.list
RUN apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE
RUN apt-get update && apt-get install -y --no-install-recommends \
	librealsense2-dkms \
	librealsense2-utils \
	librealsense2-dev \
	librealsense2-dbg

# phoxi
RUN wget -O - http://www.photoneo.com/files/PhotoneoPhoXiControl-1.1.32-Ubuntu16.04-LTS-64-gcc5.4.tar \
	| tar xf - \
	&& ./PhotoneoPhoXiControl-1.1.32-Ubuntu16.04-LTS-64-gcc5.4.run \
    && rm ./PhotoneoPhoXiControl-1.1.32-Ubuntu16.04-LTS-64-gcc5.4.run
ENV PHOXI_CONTROL_PATH /opt/PhotoneoPhoXiControl
ENV PATH ${PATH}:${PHOXI_CONTROL_PATH}/bin
ENV LD_LIBRARY_PATH ${LD_LIBRARY_PATH}:${PHOXI_CONTROL_PATH}/API/lib
ENV CPATH ${CPATH}:${PHOXI_CONTROL_PATH}/API/include
ENV PhoXi_DIR ${PHOXI_CONTROL_PATH}
COPY ./docker/cmake/PhoXiConfig.cmake /opt/PhotoneoPhoXiControl/

################################################################################

# vision-visp package used in easy handeye
RUN apt-get install -y ros-kinetic-vision-visp

# For enabling vpMatrix.eigen() in vision-visp package
RUN apt-get update && apt-get install -y --no-install-recommends \
	libgsl-dev

################################################################################
# Install packages needed by o2as_usb_relay
RUN apt-get update && apt-get install -y --no-install-recommends \
	libusb-dev \
	&& rm -rf /var/lib/apt/lists/*

# For convenience only
RUN apt-get update && apt-get install -y --no-install-recommends \
	less \
	vim \
	usbutils

################################################################################

# Install dependencies for dynamixel and serial communication.

RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-kinetic-dynamixel-sdk \
	ros-kinetic-qt-build \
	ros-kinetic-dynamixel-workbench \
    ros-kinetic-dynamixel-workbench-msgs
RUN pip install pyserial

################################################################################

# Install pymodbus for the Robotiq gripper driver

RUN apt-get update && apt-get install -y --no-install-recommends \
	python-pymodbus

################################################################################
# Install opencv for the webcam
RUN apt-get install -y cmake \
         libgtk2.0-dev \
         pkg-config \
         libavcodec-dev \
         libavformat-dev \
         libswscale-dev \
         libdc1394-22-dev \
         libjasper-dev \
         libtbb2 \
         libtbb-dev \
         libjpeg-dev \
         libpng-dev \
         libtiff-dev

RUN pip install opencv-python

################################################################################

# Downlad and install codemeter runtime
RUN wget -O /tmp/codemeter.deb https://download.ensenso.com/s/ensensosdk/download?files=codemeter_6.40.2402.501_amd64.deb
RUN dpkg -i /tmp/codemeter.deb
RUN apt-get install -f -y

################################################################################

# Install MoveIt! dependencies, because we build it from source
RUN apt-get update && apt-get install -y ros-kinetic-moveit-resources

################################################################################

# Install MATLAB dependencies to run it into docker.
RUN apt update && apt install -y --no-install-recommends \
	unzip && rm -rf /var/lib/apt/lists/*

################################################################################

# Patch for Moveit RobotCommander fails on kinetic #234
# https://github.com/shadow-robot/sr_interface/issues/234
# remove installed pyassimp and upgrade by pip
RUN dpkg --remove --force-depends python-pyassimp
RUN pip install --upgrade pip
RUN pip install --upgrade pyassimp

################################################################################

# Install python can module to run xela sensor demo.
RUN pip install python-can

################################################################################

# Define the working directory.
WORKDIR /root/

# Copy the helper scripts to the working directory.
COPY ./docker/scripts/. /root/scripts/
RUN chmod -R +x /root/scripts/*

# Setup the Bash shell environment with '/root/.bashrc'.
RUN cat /root/scripts/initialize-bash-shell.sh >> /root/.bashrc

# Overwrite the entry point of the parent image.
ENTRYPOINT []
RUN echo "\nsource /root/catkin_ws/devel/setup.bash\n" >> /root/.bashrc

# Enter the container with a Bash shell.
CMD ["/bin/bash"]

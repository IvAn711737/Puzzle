FROM nvcr.io/nvidia/l4t-base:r32.7.1

# To avoid waiting for input during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8 
ENV ROS_DISTRO melodic

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    dpkg \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654


# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-ros-core \
    && rm -rf /var/lib/apt/lists/*
    
# installing tools for ros
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    build-essential \
    python-rosdep \
    python-catkin-tools \
    python-wstool \
    git \
    python-rosinstall \
    python-vcstools \
    nano \
    libeigen3-dev \
    ros-melodic-resource-retriever \
    ca-certificates \
    openssl \
    && rm -rf /var/lib/apt/lists/*    

# copying a puzzle in docker conteiner    
COPY src /puzzle/src
WORKDIR /puzzle
RUN catkin init --workspace .

# Cloning and installing the package Librealsense
COPY .prodcontainer/scripts/build_librealsense.sh /tmp
WORKDIR /tmp
RUN chmod +x build_librealsense.sh
RUN ./build_librealsense.sh

RUN pip install transforms3d

# Cloning and installing GeographicalLib install 
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -P /tmp
RUN chmod +x install_geographiclib_datasets.sh
RUN ./install_geographiclib_datasets.sh

# check which command script is available
if hash geographiclib-get-geoids; then
	run_get geoids geoids egm96-5
	run_get gravity gravity egm96
	run_get magnetic magnetic emm2015
elif hash geographiclib-datasets-download; then # only allows install the goid model dataset
	geographiclib-datasets-download egm96_5;
else
	echo "OS not supported! Check GeographicLib page for supported OS and lib versions." 1>&2
fi


# installing tools for ros
RUN apt-get update && \
    rosdep init && \
    rosdep --rosdistro $ROS_DISTRO update && \
    rosdep install -y \
    --from-paths /puzzle/src \
    --ignore-src \
    --rosdistro ${ROS_DISTRO} \
    --as-root=apt:false \
    && rm -rf /var/lib/apt/lists/*

    
WORKDIR /puzzle   

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin build
FROM px4io/px4-dev-ros-noetic:latest
LABEL Name=puzzle Version=0.0.3

ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USER_NAME=dev

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN groupadd -g $GROUP_ID $USER_NAME
RUN useradd -rm -d /home/$USER_NAME -s /bin/bash -g $GROUP_ID -u $USER_ID $USER_NAME
RUN gpasswd -a $USER_NAME dialout
# -r, --system Create a system account. see: Implications creating system accounts
# -m, --create-home Create the user's home directory.
# -d, --home-dir HOME_DIR Home directory of the new account.
# -s, --shell SHELL Login shell of the new account.
# -g, --gid GROUP Name or ID of the primary group.
# -G, --groups GROUPS List of supplementary groups.
# -u, --uid UID Specify user ID. see: Understanding how uid and gid work in Docker containers
# -p, --password PASSWORD Encrypted password of the new account (e.g. ubuntu).
# --gecos, parameter is used to set the additional information. In this case it is just empty.

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

RUN apt update \
    && apt install software-properties-common -y

RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

RUN apt update \
    && apt install ros-noetic-rviz -y \
    && apt install ros-noetic-rqt-common-plugins -y \
    && apt install ros-noetic-rqt-robot-plugins -y \
    && apt install ros-noetic-image-geometry -y \
    && apt install librealsense2-dkms -y \
    && apt install librealsense2-utils -y \
    && apt install librealsense2-dev -y \
    && apt install ros-noetic-aruco-ros -y
RUN pip3 install pyqtgraph    
RUN pip install transforms3d

RUN echo "parse_git_branch() {" >> /home/${USER_NAME}/.bashrc \
    && echo "    git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/ (\\\1)/'" >> /home/${USER_NAME}/.bashrc \
    && echo "}" >> /home/${USER_NAME}/.bashrc \
    && echo "export PS1='\u@\h \[\033[32m\]\w\[\033[33m\]\$(parse_git_branch)\[\033[00m\] $ '" >> /home/${USER_NAME}/.bashrc

USER ${USER_NAME}
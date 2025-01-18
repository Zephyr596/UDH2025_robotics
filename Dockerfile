FROM px4io/px4-dev-ros-noetic AS base
ENV DEBIAN_FRONTEND=noninteractive
SHELL [ "/bin/bash", "-o", "pipefail", "-c" ]

ARG UNAME=sim
ARG USE_NVIDIA=1

# Dependencies
RUN sudo apt-get update \
    && apt-get install -y -qq --no-install-recommends \
    python-is-python3 \
    apt-utils \
    byobu \
    fuse \
    git \
    libxext6 \
    libx11-6 \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libfuse-dev \
    libpulse-mainloop-glib0 \
    rapidjson-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \ 
    gstreamer1.0-gl \
    iputils-ping \
    nano \
    vim \
    sed \
    wget \
    gz-garden \
    && rm -rf /var/lib/apt/lists/*

# Python deps
RUN sudo pip install PyYAML MAVProxy

# User
RUN adduser --disabled-password --gecos '' $UNAME
RUN adduser $UNAME sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
ENV HOME=/home/$UNAME
USER $UNAME

# ROS vars
RUN echo "export GZ_VERSION='garden'" >> ~/.bashrc

ENV DISPLAY=:0
RUN echo $DISPLAY

WORKDIR $HOME

# Load packages
ARG PX4_TAG="v1.15.2"
RUN sudo git clone --depth 1 --branch $PX4_TAG --recurse-submodules https://github.com/PX4/PX4-Autopilot.git
RUN git config --global --add safe.directory /home/sim/PX4-Autopilot
# Add wadibirk world file
ADD ./world/wadibirk.sdf /home/sim/PX4-Autopilot/Tools/simulation/gz/worlds/wadibirk.sdf
ADD /catkin_ws/src/sitl_targets_gazebo-classic.cmake /home/sim/PX4-Autopilot/Tools/simulation/gz/worlds/sitl_targets_gazebo-classic.cmake
RUN sudo sed -i '/set(gz_worlds/,/)/ s/)/ wadibirk)/' /home/sim/PX4-Autopilot/src/modules/simulation/gz_bridge/CMakeLists.txt
# export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models:~/PX4-Autopilot/Tools/simulation/gz/worlds
# Apply patch according to https://github.com/PX4/PX4-Autopilot/pull/21617
# WORKDIR /home/sim/PX4-Autopilot
# ADD ./patch/fix.patch /tmp/fix.patch
# RUN sudo git apply --whitespace=fix /tmp/fix.patch

# ROS vars
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source ~/UDH2025_robotics/catkin_ws/devel/setup.bash --extend" >> ~/.bashrc

# Nvidia GPU vars
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
RUN if [[ -z "${USE_NVIDIA}" ]] ;\
    then printf "export QT_GRAPHICSSYSTEsudo docker imagesM=native" >> /home/${UNAME}/.bashrc ;\
    else echo "Native rendering support disabled" ;\
    fi

# fix OpenGL bug according to https://github.com/microsoft/WSL/issues/7507#issuecomment-1698412148
ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib
ENV LIBVA_DRIVER_NAME=d3d12
# Add terminal commands
RUN echo "alias copy_files='sudo cp /home/sim/UDH2025_robotics/world/wadibirk.sdf /home/sim/UDH2025_robotics/catkin_ws/src/sitl_targets_gazebo-classic.cmake /home/sim/PX4-Autopilot/Tools/simulation/gz/worlds'" >> ~/.bashrc && \
    echo "alias make_px4='cd /home/sim/PX4-Autopilot && make px4_sitl gz_x500'" >> ~/.bashrc
    echo "alias run='/home/sim/UDH2025_robotics/run_multiple.sh 4'" >> ~/.bashrc && \
    echo "alias fly='roslaunch drones_sim px4_sim_recursed.launch n:=4'" >> ~/.bashrc && \
    echo "alias rm_log='rm -rf ~/.ros'" >> ~/.bashrc
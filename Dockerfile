#iinstall ros package
# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:humble-ros-core-jammy

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros2 packages
RUN apt-get update && apt-get install -y --install-recommends ros-dev-tools && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \        
     apt-get install -y git


#RUN ["chmod", "+x", "/opt/ros/humble/setup.sh"]

#RUN /opt/ros/humble/setup.sh

RUN cd home && git clone https://github.com/Ararabots-UFMS/ssl-VICE.git

RUN cd /home/ssl-VICE && . /opt/ros/$ROS_DISTRO/setup.sh && colcon build

RUN echo 'source /home/ssl-VICE/install/local_setup.sh' >> ~/.bashrc

RUN apt install -y python3-pip

RUN cd home/ssl-VICE && pip install -r requirements.txt


#RUN cd /home/ssl-VICE/ && colcon build

RUN echo 'export ARARA_VICE_PATH = path/to/repo' >> ~/.bashrc

RUN cd home && git clone https://github.com/Ararabots-UFMS/ssl-gui.git

#RUN npm install

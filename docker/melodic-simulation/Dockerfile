FROM osrf/ros:melodic-desktop-full

RUN apt-get update && \
    apt-get install -y gnome-panel gnome-settings-daemon metacity nautilus gnome-terminal && \
    apt-get install -y tightvncserver && \
	apt-get install -y wget && \
	apt-get install -y tar && \
	apt-get install ros-melodic-catkin python-catkin-tools 

# Additional Dependencies (for rospy python3)
RUN sudo apt-get install -y python3-pip python3-yaml
RUN sudo pip3 install rospkg catkin_pkg

RUN sudo apt install nano
RUN pip3 install numpy
RUN pip3 install matplotlib
RUN sudo apt-get install -y python3-tk
RUN sudo apt install -y python-pip
RUN sudo apt-get install -y python-tk
RUN pip3 install dill
RUN pip3 install more-itertools

ARG UNAME=user
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID -o $UNAME
RUN useradd -m -u $UID -g $GID -o -s /bin/bash $UNAME
USER $UNAME
WORKDIR "/home/user/"
RUN mkdir ari_public_ws
WORKDIR "/home/user/ari_public_ws"
RUN wget https://raw.githubusercontent.com/pal-robotics/ari_tutorials/master/ari_public-melodic.rosinstall

RUN rosinstall src /opt/ros/melodic ari_public-melodic.rosinstall
RUN rosdep update

USER root
RUN rosdep install --from-paths src --ignore-src --rosdistro melodic --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup opencv3 librealsense2-dev librealsense2-dkms hey5_transmissions ydlidar_ros_driver" -y

WORKDIR "/home/user/ari_public_ws"
RUN ["chmod", "+x", "/opt/ros/melodic/setup.bash"]
RUN bash -c "source /opt/ros/melodic/setup.bash \
		&& catkin build -DCMAKE_BUILD_TYPE=Release \
		&& echo 'source /home/user/ari_public_ws/devel/setup.bash' >> ~/.bashrc"
RUN apt update && apt install vim -y
USER $UNAME
WORKDIR "/home/user/"

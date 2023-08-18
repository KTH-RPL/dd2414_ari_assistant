FROM osrf/ros:noetic-desktop-full

RUN apt update
RUN apt install -y wget
RUN apt install -y tar
RUN apt install -y ros-noetic-catkin python3-catkin-tools 

# Additional Dependencies (for rospy python3)
RUN apt install -y python3-pip python3-yaml
RUN pip3 install rospkg catkin_pkg

RUN apt install -y nano
RUN apt install -y python3-tk
RUN apt install -y vim
RUN apt install -y git
RUN pip3 install numpy
RUN pip3 install matplotlib
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
COPY ari_public-melodic.rosinstall ari_public-melodic.rosinstall

RUN rosinstall src /opt/ros/noetic ari_public-melodic.rosinstall
RUN rosdep update

USER root
RUN rosdep install --from-paths src --ignore-src --rosdistro noetic --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup opencv3 librealsense2-dev librealsense2-dkms hey5_transmissions ydlidar_ros_driver" -y

USER $UNAME
WORKDIR "/home/user/ari_public_ws"
RUN bash -c "source /opt/ros/noetic/setup.bash \
		&& catkin build -DCMAKE_BUILD_TYPE=Release \
		&& echo 'source /home/user/ari_public_ws/devel/setup.bash' >> ~/.bashrc"
WORKDIR "/home/user/"
USER root
RUN apt install python-is-python3
USER $UNAME
RUN cp /home/user/ari_public_ws/src/pal_navigation_cfg_public/pal_navigation_cfg_ari/config/state_machine/state_machine_public_sim.yaml /home/user/ari_public_ws/src/pal_navigation_cfg_public/pal_navigation_cfg_ari/config/state_machine/state_machine_public_sim_laser.yaml
RUN cp /home/user/ari_public_ws/src/pal_navigation_cfg_public/pal_navigation_cfg_ari/config/base/common/global_costmap_plugins_public_sim.yaml /home/user/ari_public_ws/src/pal_navigation_cfg_public/pal_navigation_cfg_ari/config/base/common/global_costmap_plugins_public_sim_laser.yaml
RUN cp /home/user/ari_public_ws/src/pal_navigation_cfg_public/pal_navigation_cfg_ari/config/base/common/global_costmap_public_sim.yaml /home/user/ari_public_ws/src/pal_navigation_cfg_public/pal_navigation_cfg_ari/config/base/common/global_costmap_public_sim_laser.yaml
RUN cp /home/user/ari_public_ws/src/pal_navigation_cfg_public/pal_navigation_cfg_ari/config/base/common/local_costmap_plugins_public_sim.yaml /home/user/ari_public_ws/src/pal_navigation_cfg_public/pal_navigation_cfg_ari/config/base/common/local_costmap_plugins_public_sim_laser.yaml
RUN cp /home/user/ari_public_ws/src/pal_navigation_cfg_public/pal_navigation_cfg_ari/config/base/common/local_costmap_public_sim.yaml /home/user/ari_public_ws/src/pal_navigation_cfg_public/pal_navigation_cfg_ari/config/base/common/local_costmap_public_sim_laser.yaml
COPY navigation.launch /home/user/ari_public_ws/src/ari_simulation/ari_2dnav_gazebo/launch/navigation.launch

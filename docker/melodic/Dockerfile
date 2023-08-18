FROM registry.gitlab.com/pal-robotics/ari-0622/dockers/pal-ferrum-pal-metapkg-development-ari-30-dev:latest

RUN apt update && apt install -y gedit
RUN echo "source /opt/ros/melodic/setup.bash" >> /home/user/.bashrc
RUN echo "source /opt/pal/ferrum/setup.bash" >> /home/user/.bashrc
RUN echo "alias navigation=\"rosrun rviz rviz -d \$(rospack find ari_2dnav)/config/rviz/navigation.rviz\"" >> /home/user/.bashrc
WORKDIR "/home/user/"

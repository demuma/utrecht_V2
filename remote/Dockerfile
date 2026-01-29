FROM ros:humble

RUN : \
    && apt-get update \
    && apt-get install --assume-yes \
        gettext \
        python3-pip \
        ros-${ROS_DISTRO}-mqtt-client \
    && apt-get clean \
    && pip3 install flask

COPY ros_entrypoint.sh .

WORKDIR /root/ros2_ws

COPY . .

RUN colcon build

EXPOSE 80

CMD ["ros2", "launch", "remote", "remote_launch.py"]

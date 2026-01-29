FROM ros:humble

RUN : \
    && apt-get update \
    && apt-get install --assume-yes \
        gettext \
        python3-serial \
        ros-${ROS_DISTRO}-mqtt-client \
    && apt-get clean

COPY ros_entrypoint.sh .

WORKDIR /root/ros2_ws

COPY src src

RUN colcon build

CMD ["ros2", "launch", "robotic_arm", "launch.py"]

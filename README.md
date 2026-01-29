# Docker Compose setup for ROS component

This setup can be used to remotely control a robotic arm via ROS over MQTT.
You should be able to operate the motor and servos using the web interface.

### Remotely controlled by HAW Hamburg

HU should start the `robot` service on a machine that is connected to the
robotic arm: 

```bash
make robot
```

The controlling entity (HAW Hamburg) should start a correctly configured
`mqtt_client` within their ROS network. The `remote` example service shows how
this can be done.

HAW Hamburg should provide their own implementation of the `remote` that
sends `Float32` messages to the following topics:

- `/robot/motor_1`
- `/robot/servo_1`
- `/robot/servo_2`
- `/robot/servo_3`
- `/robot/servo_4`

# Getting started

#### Clone the repository
```bash
git clone https://github.com/demuma/utrecht
```

#### Build the Docker image
```bash
cd utrecht_V2
docker build . -t utrecht_V2
```

#### Run the Docker
```bash
docker run -it utrecht_V2
```

#### Run the Docker with X11
```bash
docker run -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1"\
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --network=host\
    utrecht_V2
```

#### Start the MQTT client
```bash
ros2 launch mqtt_client standalone.launch.xml params_file:=/root/Documents/bridge.yaml &
```

## Sending and receiving ROS messages
### Example 1: Set hydraulic pump throttle to full
```bash
ros2 topic pub /robot/motor std_msgs/msg/Float32 "data: 1200" -1
```

### Example 2: Move servo 1 continuously and get angle
```bash
ros2 topic pub /robot/servo_1 std_msgs/msg/Float32 "data: 1050" -1
ros2 topic echo /robot/beta_1
```

### Example 3: Move servo 2 and 3 continuously
```bash
ros2 topic pub /robot/servo_2 std_msgs/msg/Float32 "data: 1050" -1
ros2 topic pub /robot/servo_3 std_msgs/msg/Float32 "data: 1050" -1
ros2 topic echo /robot/beta_2 --once
ros2 topic echo /robot/beta_3 --once
```

### Example 4: Use the provided Python script
```bash
cd /root/ros2_ws/task
python task.py
```

## Re-entering the Docker container
#### Show all last container IDs
```bash
docker ps -a
```

#### Find container ID (most recent ID)
```bash
CONTAINER_ID=`docker ps -a | awk 'NR==2 {print $1}'`
```

#### Start Docker container
```bash
docker start $CONTAINER_ID
```

#### Enter Docker container
```bash
docker exec -it "$CONTAINER_ID" /bin/bash
```

#### Source ROS2
```bash
source /opt/ros/humble/setup.bash
```

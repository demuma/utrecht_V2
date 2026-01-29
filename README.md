# Docker Compose setup for ROS component

This setup can be used to remotely control a robotic arm via ROS over MQTT.

## Prerequisites

You need a publicly accessible MQTT broker to use this setup. You can create a
free instance at [CloudAMQP.com] or use an existing broker if you have one.

To create a free CloudAMQP instance, follow these steps:

1. Create an account at [CloudAMQP.com].
2. From the main dashboard, click on "Create New Instance".
3. Choose the free “Little Lemur” plan (about halfway in the dropdown list).
4. Select the nearest AWS region to your location. Ideally EU-Central-1
   (Frankfurt), but otherwise EU-West-1 (Ireland) or EU-North-1 (Stockholm) is
   also fine.
5. Complete the setup and wait for the instance to be created.
6. Back on the main dashboard, click on the name of your newly created instance.
7. Scroll down to “MQTT details” and note down the following information:
   - Hostname (=`MQTT_HOST`)
   - Port for TLS, usually `8883` (=`MQTT_PORT`)
   - Username, should look like `abcdefgh:abcdefgh` (=`MQTT_USER`)
   - Password (=`MQTT_PASS`)

[CloudAMQP.com]: https://www.cloudamqp.com/

## Getting started

This Docker Compose setup consists of several services, which are designed
to run separately (although they can also be started on the same machine in
different terminal sessions).

| Service  | Purpose                                                                                                                                                                                                            |
|----------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `remote` | This service provides a web interface that can be used to control the arm and view live telemetry. In reality this service would be implemented by HAW Hamburg students. The controller includes an `mqtt-client`. |
| `robot`  | This service receives commands from “Hamburg” (`remote`) and converts them into MAVROS commands that move the robotic arm. Platforms other than Linux (either native or VM) are not supported.                     |

You should set the credentials for the MQTT broker before running any service.

```bash
export MQTT_HOST=<insert host here>
export MQTT_PORT=<insert port here>
export MQTT_USER=<insert user here>
export MQTT_PASS=<insert pass here>
```

### Local development and testing

Run the following command to start the services on the controlling side. This
includes an example script that starts an admin web server on http://localhost/
and an MQTT “spy” that is configured to automatically forward messages to an
MQTT broker.

```bash
make remote
```

Then, run the following command on another machine:

```bash
make robot
```

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
cd /root/Documements
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

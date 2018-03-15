# drone-facial-tracking
A ROS package intended for quadcopter facial tracking, using a Parrot Drone 2.0. ROS Kinetic was used, with python nodes, implementing OpenCV3 and Dlib for detection and tracking capabilities, respectively. The drone is accessed and controlled using the ardrone-autonomy ROS driver (through TCP and UDP Ports).

Currently, the drone only follows horizontal movements, for simplicity, which are smoothened using the ROS pid node, with only a proportional term for now. This node can be installed with the following command:

sudo apt-get install ros-kinetic-pid   (replace 'kinetic' with corresponding ROS release)

Optimal PID terms will be added soon, as well as support for vertical tracking movements.

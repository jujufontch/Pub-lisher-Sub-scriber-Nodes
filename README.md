Nodes with Pulisher and Subsciber 


talker listens to two ROS parameters v and d.
talker publishes an AckermannDriveStamped message with the speed field equal to the v parameter and steering_angle field equal to the d parameter, and to a topic named drive.
talker publishes as fast as possible.
To test node, set the two ROS parameters through command line, a launch file, or a yaml file.
The second node will be named relay.cpp or relay.py and needs to meet these criteria:

relay subscribes to the drive topic.
In the subscriber callback, take the speed and steering angle from the incoming message, multiply both by 3, and publish the new values via another AckermannDriveStamped message to a topic named drive_relay.

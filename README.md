NODES WITH PUBLISHER AND SUBSCRIBER


-Talker listens to two ROS parameters v and d.
-Talker publishes an AckermannDriveStamped message with the speed field equal to the v parameter and steering_angle field equal to the d parameter, and to a topic named drive.
-Talker publishes as fast as possible.

-Relay subscribes to the drive topic.
-In the subscriber callback,  the speed and steering angle from the incoming message are both multiplied by 3, and  the new values are published  via another AckermannDriveStamped message to a topic named drive_relay.

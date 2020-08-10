# TrackBots
Tracking software to track robots detected by [SSL Vision](https://github.com/RoboCup-SSL/ssl-vision).
This fork modifies the TrackBots code to publish robot position and velocity data as ROS [odom messages](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html)
instead of protobuf serialized messages sent as udp packets. Field geometry, ball information, and robot team information are ignored.

SSL Vision provides the following robot position information (positions are in the [SSL Vision coordinate frame](https://raw.githubusercontent.com/wiki/RoboCup-SSL/ssl-vision/images/ssl-vision-cameras.png)): x, y, orientation. 
A Kalman filter is applied to compute the linear and angular velocities. The ROS node publishes odom messages to a
different topic for each detected robot.

The node can be launched by executing `python start_tracker.py` and has been successfully tested with ROS melodic and python 2.7.

Original code is found in the [robocup branch](https://github.com/rdesc/TrackBots/tree/robocup).
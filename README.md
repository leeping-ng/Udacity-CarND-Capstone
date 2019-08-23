# Udacity-CarND-Capstone
This project uses ROS nodes to implement perception, planning and control for a self driving car.

The system architecture showing the ROS nodes and topics is shown below:
<img src='https://github.com/leeping-ng/Udacity-CarND-Capstone/blob/master/imgs/System%20Architecture.JPG'>

These are the C++ nodes that were updated for this project: 

### Waypoint Updater Node
The purpose of this node is to update the target velocity property of each waypoint based on traffic light and obstacle detection data. This node will subscribe to the /base_waypoints, /current_pose, /obstacle_waypoint, and /traffic_waypoint topics, and publish a list of waypoints ahead of the car with target velocities to the /final_waypoints topic

### DBW Node
The dbw_node subscribes to the /current_velocity topic along with the /twist_cmd topic to receive target linear and angular velocities. Additionally, this node will subscribe to /vehicle/dbw_enabled, which indicates if the car is under dbw or driver control. This node will publish throttle, brake, and steering commands to the /vehicle/throttle_cmd, /vehicle/brake_cmd, and /vehicle/steering_cmd topics.

### Traffic Light Detection Node
This node takes in data from the /image_color, /current_pose, and /base_waypoints topics and publishes the locations to stop for red traffic lights to the /traffic_waypoint topic.

The /current_pose topic provides the vehicle's current position, and /base_waypoints provides a complete list of waypoints the car will be following.

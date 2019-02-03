# optic_flow
## 1. Overview
An open source implementation of a generic optical flow sensor. This will be following one of my personal projects, and if it goes well I will publish the decoupled code.
## 2. Nodes
All noses subscribing to [sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html) topics use [image_transport](http://wiki.ros.org/image_transport).
### 2.3 flow_node
This node is the main node of the package. It takes in images and returns a position estimate of the robot calculated from optical flow data.
#### 2.3.1 Subscribed Topics

 - image [sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)
	 - The image topic of interest.
#### 2.3.2 Published Topics
 - vel [geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html)
	 - The computed velocity of the sensor.

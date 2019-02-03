# optic_flow
## 1. Overview
An open source implementation of a generic optical flow sensor. This will be following one of my personal projects, and if it goes well I will publish the decoupled code.
## 2. Nodes
All noses subscribing to [sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html) topics use [image_transport](http://wiki.ros.org/image_transport).
### 2.3 flow_node
This node is the main node of the package. It takes in images and returns a position estimate of the robot calculated from optical flow data.


<!--stackedit_data:
eyJoaXN0b3J5IjpbNjE2NzcxNTU2LC0xMDk3NzM0ODIzXX0=
-->
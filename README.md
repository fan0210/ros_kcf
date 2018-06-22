# ros_kcf
A ros wrapper for Kernel Correlation Filter tracker

## Building ros_kcf
### 1.Prerequisites
* [ROS](http://wiki.ros.org/ROS/Installation)
* [msgs pkg](https://github.com/FanKaii/ros_people_detect/tree/master/msgs)
* [test pkg](https://github.com/FanKaii/ros_people_detect/tree/master/people_detect_test)

### 2.Create a workspace and compile
`mkdir -p ~/catkin_ws/src`<br>
next, copy these three packages (ros_kcf,msgs,people_detect_test) to `/catkin_ws/src` and<br>
`catkin_make`<br>

## Usage 
* Set your own image topic

  find [ros_kcf_node.launch](https://github.com/FanKaii/ros_kcf/blob/master/ros_kcf/launch/ros_kcf_node.launch) and [people_detect_test.launch](https://github.com/FanKaii/ros_people_detect/blob/master/people_detect_test/launch/people_detect_test.launch) and replace `/image_pub/image` with your own image topic.

* Run ros_kcf node

  `roslaunch ros_kcf ros_kcf_node.launch`
  
* Run people_detect_test node

  `roslaunch people_detect_test people_detect_test.launch`
  
## Show results

  ![img1 load error](https://github.com/FanKaii/ros_people_detect/blob/master/image/img2.png)

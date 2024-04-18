### Gazebo simulation website
```
https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation
```
### ActionLib 
```
$ cd %YOUR_CATKIN_WORKSPACE%/src
$ catkin_create_pkg actionlib_tutorials actionlib message_generation roscpp rospy std_msgs actionlib_msgs
```

### Actions using Goals
```
https://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28GoalCallbackMethod%29
```

```
To manually generate the message files from this file:


$ roscd actionlib_tutorials
$ rosrun actionlib_msgs genaction.py -o msg/ action/Averaging.action

```

```
To automatically generate the message files during the make process, add the following to CMakeLists.txt:

find_package(catkin REQUIRED COMPONENTS actionlib std_msgs message_generation) 
add_action_files(DIRECTORY action FILES Averaging.action)
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)
```

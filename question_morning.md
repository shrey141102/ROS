To achieve this task, you'll need to create a ROS node that sends a goal to an action server and listens for feedback and results. Below is the Python code for the client node, along with the necessary changes to `package.xml` and `CMakeLists.txt`:

1. **Create the Python client node (`client_node.py`)**:

```python
#!/usr/bin/env python

import rospy
import actionlib
from my_package.msg import MyAction, MyGoal, MyFeedback, MyResult

def feedback_callback(feedback):
    rospy.loginfo("Feedback received: {}".format(feedback.progress))

def send_goal(n, m):
    client = actionlib.SimpleActionClient('my_action', MyAction)
    client.wait_for_server()

    goal = MyGoal(n=n, m=m)
    client.send_goal(goal, feedback_cb=feedback_callback)

    client.wait_for_result()
    result = client.get_result()
    
    if result.success:
        rospy.loginfo("Goal reached: {}".format(result.success))
    else:
        rospy.loginfo("Failed to reach goal: {}".format(result.success))

if __name__ == '__main__':
    rospy.init_node('client_node')
    n = 20  # Example values for n and m
    m = 2
    send_goal(n, m)
```

2. **Modify `package.xml`**:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>my_package</name>
  <version>0.0.0</version>
  <description>The my_package package</description>

  <maintainer email="user@example.com">user</maintainer>

  <license>Apache License 2.0</license>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>actionlib</build_depend>
  <build_depend>actionlib_msgs</build_depend>

  <run_depend>rospy</run_depend>
  <run_depend>std_msgs</run_depend>
  <run_depend>actionlib</run_depend>
  <run_depend>actionlib_msgs</run_depend>
  
  <export>
  </export>
</package>
```

3. **Modify `CMakeLists.txt`**:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(my_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs actionlib actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
```

4. **Commands in Ubuntu**:

Assuming your package is named `my_package`:

- **To create the action message files**:

```bash
cd ~/catkin_ws/src/my_package
mkdir action
touch My.action
```

Open `My.action` and add:

```
# Define the goal
int32 n
int32 m

# Define the result
bool success

---
# Define the feedback
int32 progress
```

- **Add the necessary dependencies to `package.xml` and `CMakeLists.txt` as shown above.**

- **Build your package**:

```bash
cd ~/catkin_ws
catkin_make
```

- **Source your workspace**:

```bash
source devel/setup.bash
```

- **Run the ROS master**:

```bash
roscore
```

- **Run your server node**:

```bash
rosrun my_package server_node.py
```

- **Run your client node**:

```bash
rosrun my_package client_node.py
```

This setup assumes you have a ROS package named `my_package` and you've defined a custom action message `My.action` with the appropriate fields. Adjust the package name and message file name as needed.
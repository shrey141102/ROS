Sure! Let's create an example where the action server and client play a guessing game. The client sends a guess (an integer) to the server, and the server responds with feedback indicating if the guess is too high or too low. Once the correct guess is made, the server returns a success message.

Here's how you can do it:

**1. Create a Workspace:**
Create a workspace and a package to work in. Use the following commands:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_create_pkg guessing_game_action actionlib message_generation roscpp std_msgs
```

**2. Define the Action:**
Define the action in a `.action` file in the `action` directory of your package. The file for the guessing game action might look like this:
```bash
#goal definition
int32 guess
---
#result definition
bool success
---
#feedback
string feedback
```
Create this file with the following commands:
```bash
cd ~/catkin_ws/src/guessing_game_action
mkdir action
echo -e "int32 guess\n---\nbool success\n---\nstring feedback" > action/GuessingGame.action
```

**3. Modify CMakeLists.txt and package.xml:**
Ensure that `actionlib_msgs` is a dependency in your `package.xml` and that your `CMakeLists.txt` is set up to generate messages from your `.action` file.

In `package.xml`, make sure these lines exist:
```xml
<build_depend>actionlib_msgs</build_depend>
<exec_depend>actionlib_msgs</exec_depend>
```

In `CMakeLists.txt`, make sure these lines exist:
```cmake
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
)

add_action_files(
  DIRECTORY action
  FILES GuessingGame.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)
```

**4. Write the Action Server and Client:**
Write the action server and client. The server generates a random number and compares it with the guess sent by the client, providing feedback. The client continues to send guesses until the correct one is made.

**5. Build and Source Your Workspace:**
After writing your action server and client, build your workspace with the following commands:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

**6. Run the Action Server and Client:**
Run the action server and client. In one terminal, run the server:
```bash
rosrun guessing_game_action guessing_game_server
```
And in another terminal, run the client:
```bash
rosrun guessing_game_action guessing_game_client
```

These steps should guide you through creating an action server and client in ROS Noetic for playing a guessing game using only integers and strings. Let me know if you have any questions! 😊


Sure! Here's the Python implementation for the server and client:

**Server Code (guessing_game_server.py):**
```python
#!/usr/bin/env python

import rospy
import actionlib
from guessing_game_action.msg import GuessingGameAction, GuessingGameGoal, GuessingGameResult

class GuessingGameServer(object):
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, GuessingGameAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        success = True
        feedback = GuessingGameResult()
        target_number = rospy.get_param("~target_number", 42) # Default target number

        rospy.loginfo("{}: Executing, target number is {}".format(self._action_name, target_number))

        for attempts in range(10):
            if self._as.is_preempt_requested():
                rospy.loginfo("{}: Preempted".format(self._action_name))
                self._as.set_preempted()
                success = False
                break

            if goal.guess < target_number:
                feedback.feedback = "Too low!"
                self._as.publish_feedback(feedback)
                rospy.loginfo("{}: Too low!".format(self._action_name))
            elif goal.guess > target_number:
                feedback.feedback = "Too high!"
                self._as.publish_feedback(feedback)
                rospy.loginfo("{}: Too high!".format(self._action_name))
            else:
                result = GuessingGameResult()
                result.success = True
                rospy.loginfo("{}: Succeeded".format(self._action_name))
                self._as.set_succeeded(result)
                break

            rospy.sleep(1)

        if not success:
            result = GuessingGameResult()
            result.success = False
            self._as.set_aborted(result)

if __name__ == '__main__':
    rospy.init_node('guessing_game_server')
    server = GuessingGameServer(rospy.get_name())
    rospy.spin()
```

**Client Code (guessing_game_client.py):**
```python
#!/usr/bin/env python

import rospy
import actionlib
from guessing_game_action.msg import GuessingGameAction, GuessingGameGoal

def done_cb(status, result):
    rospy.loginfo("Game Finished!")
    if result.success:
        rospy.loginfo("Congratulations! You guessed it right!")
    else:
        rospy.loginfo("Oops! Out of attempts. Better luck next time!")
    rospy.signal_shutdown("Game Over")

def active_cb():
    rospy.loginfo("Goal just went active")

def feedback_cb(feedback):
    rospy.loginfo("Feedback: {}".format(feedback.feedback))

def main():
    rospy.init_node('guessing_game_client')

    if len(rospy.myargv()) != 2:
        rospy.loginfo("Usage: guessing_game_client.py <guess>")
        return

    guess = int(rospy.myargv()[1])

    client = actionlib.SimpleActionClient('guessing_game', GuessingGameAction)
    rospy.loginfo("Waiting for action server to start.")
    client.wait_for_server()
    rospy.loginfo("Action server started, sending goal.")

    goal = GuessingGameGoal()
    goal.guess = guess

    client.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)

    rospy.spin()

if __name__ == '__main__':
    main()
```

You can save these Python scripts in your package's `src` directory. Ensure that you have the `guessing_game_action` package installed or available in your workspace. Then, you can run the server and client using the same instructions provided earlier. Let me know if you need further assistance!

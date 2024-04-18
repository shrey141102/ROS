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



Certainly! Here's the server and client code for the guessing game example:

**Server Code (guessing_game_server.cpp):**
```cpp
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <guessing_game_action/GuessingGameAction.h>
#include <cstdlib>
#include <ctime>

class GuessingGameServer {
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<guessing_game_action::GuessingGameAction> as_;
  std::string action_name_;
  guessing_game_action::GuessingGameFeedback feedback_;
  guessing_game_action::GuessingGameResult result_;

public:
  GuessingGameServer(std::string name) :
    as_(nh_, name, boost::bind(&GuessingGameServer::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
    srand(time(0));
  }

  ~GuessingGameServer(void) {}

  void executeCB(const guessing_game_action::GuessingGameGoalConstPtr &goal)
  {
    ros::Rate r(1);
    bool success = true;
    int32_t target_number = rand() % 100 + 1;

    ROS_INFO("%s: Executing, target number is %d", action_name_.c_str(), target_number);

    for (int attempts = 0; attempts < 10; attempts++) {
      if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
        success = false;
        break;
      }

      if (goal->guess < target_number) {
        feedback_.feedback = "Too low!";
        as_.publishFeedback(feedback_);
        ROS_INFO("%s: Too low!", action_name_.c_str());
      } else if (goal->guess > target_number) {
        feedback_.feedback = "Too high!";
        as_.publishFeedback(feedback_);
        ROS_INFO("%s: Too high!", action_name_.c_str());
      } else {
        result_.success = true;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_);
        break;
      }

      r.sleep();
    }

    if (!success) {
      result_.success = false;
      as_.setAborted(result_);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "guessing_game_server");

  GuessingGameServer server("guessing_game");
  
  ros::spin();

  return 0;
}
```

**Client Code (guessing_game_client.cpp):**
```cpp
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <guessing_game_action/GuessingGameAction.h>

typedef actionlib::SimpleActionClient<guessing_game_action::GuessingGameAction> Client;

void doneCallback(const actionlib::SimpleClientGoalState& state,
                  const guessing_game_action::GuessingGameResultConstPtr& result)
{
  ROS_INFO("Game Finished!");
  if (result->success)
    ROS_INFO("Congratulations! You guessed it right!");
  else
    ROS_INFO("Oops! Out of attempts. Better luck next time!");
  ros::shutdown();
}

void activeCallback()
{
  ROS_INFO("Goal just went active");
}

void feedbackCallback(const guessing_game_action::GuessingGameFeedbackConstPtr& feedback)
{
  ROS_INFO("Feedback: %s", feedback->feedback.c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "guessing_game_client");

  if (argc != 2) {
    ROS_INFO("Usage: guessing_game_client <guess>");
    return 1;
  }

  int32_t guess = atoi(argv[1]);

  Client client("guessing_game", true); // true -> don't need ros::spin()

  ROS_INFO("Waiting for action server to start.");
  client.waitForServer();
  ROS_INFO("Action server started, sending goal.");

  guessing_game_action::GuessingGameGoal goal;
  goal.guess = guess;

  client.sendGoal(goal, &doneCallback, &activeCallback, &feedbackCallback);

  ros::spin();

  return 0;
}
```

You can compile these files by adding them to the CMakeLists.txt of your package and then running `catkin_make`. After compilation, you can run the server and client using the instructions provided in the previous message. Let me know if you have any questions!

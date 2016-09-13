#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_template_alignment_plugin/PointcloudAlignmentAction.h>
#include <csignal>

using namespace std;

class PointcloudAlignmentClient
{
    public:
    
};

static /*volatile*/ int sigint = 0;

void signalHandler( int signum )
{
    cout<<"sigint!"<<endl;
    sigint = 1;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_object_template_alignment_plugin");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<object_template_alignment_plugin::PointcloudAlignmentAction> ac("pointcloud_alignment", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  signal(SIGINT, signalHandler);

  ROS_INFO("Action server started, sending goal.");



  // send a goal to the action
  while (sigint == 0) {
      std::cout<<"Press [Enter] to send start command to server";
      cin.get();

      object_template_alignment_plugin::PointcloudAlignmentGoal goal;
      goal.command = 0;
      ac.sendGoal(goal);
  }
  cout<<"schluss jetzt"<<endl;

  //wait for the action to return
 /* bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");*/

  //exit
  return 0;
}

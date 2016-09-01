#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <object_template_alignment_plugin/PointcloudAlignmentAction.h>
#include <geometry_msgs/PoseStamped.h>

class PointcloudAlignmentAction
{
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<object_template_alignment_plugin::PointcloudAlignmentAction> as_; 
	std::string action_name_;
	object_template_alignment_plugin::PointcloudAlignmentFeedback feedback_;
	object_template_alignment_plugin::PointcloudAlignmentResult result_;
	
public:
	PointcloudAlignmentAction(std::string name) :
	as_(nh_, name, boost::bind(&PointcloudAlignmentAction::executeCB, this, _1), false),
	action_name_(name) {
		as_.start();
	}

	~PointcloudAlignmentAction(void) {}
  
    void executeCB(const object_template_alignment_plugin::PointcloudAlignmentGoalConstPtr &goal)
    {
        // helper variables
        ros::Rate r(1);
        bool success = true;

        // push_back the seeds for the fibonacci sequence
        feedback_.percentage = 0;

        // publish info to the console for the user
        ROS_INFO("%s: Executing aligning template", action_name_.c_str());

        // start executing the action
        for(int i=1; i<=goal->command; i++)
        {
          // check that preempt has not been requested by the client
          if (as_.isPreemptRequested() || !ros::ok())
          {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
            break;
          }
          feedback_.percentage = (float(i)/float(goal->command))*100;
          // publish the feedback
          as_.publishFeedback(feedback_);
          // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
          r.sleep();
        }

        if(success)
        {
          geometry_msgs::PoseStamped result;
          geometry_msgs::Quaternion orientation;
          result.pose.orientation = orientation;
          result_.transformation_matrix = result;
          ROS_INFO("%s: Succeeded", action_name_.c_str());
          // set the action state to succeeded
          as_.setSucceeded(result_);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_alignment");

    PointcloudAlignmentAction pointcloud_alignment(ros::this_node::getName());
    ros::spin();

	return 0;
}

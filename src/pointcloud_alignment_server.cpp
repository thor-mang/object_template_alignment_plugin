#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <object_template_alignment_plugin/PointcloudAlignmentAction.h>

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
  
	void executeCB(const object_template_alignment_plugin::PointcloudAlignmentGoalConstPtr &goal) {}
};

int main(int argc, char** argv) {
	std::cout<<"blabla"<<std::endl; 
	//ros::init(argc, argv, "pointcloud alignment");

	//PointcloudAlignmentAction pointcloud_alignment(ros::this_node::getName());
	//PointcloudAlignmentAction pointcloud_alignment(ros::this_node::getName());
	
	//ros::spin();

	return 0;
}

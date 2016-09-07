#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <object_template_alignment_plugin/PointcloudAlignmentAction.h>
#include <geometry_msgs/PoseStamped.h>

#include <string>
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <stdlib.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

using namespace Eigen;
using namespace std;

MatrixXd getTemplatePointcloud(string path, string filename);

class PointcloudAlignmentAction
{
private:
	static string currentTemplate;
	static VectorXd currentPosition;
	
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
  
    void executeCB(const object_template_alignment_plugin::PointcloudAlignmentGoalConstPtr &goal) {
		bool success = true;
		
		std::string filename = "plug.pcd";
		std::string path = "vigir/vigir_templates/vigir_template_library/object_library/tools/";
		
		MatrixXd template_pointcloud = getTemplatePointcloud(path, filename);
		
		cout<<template_pointcloud<<endl;
		
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
        // helper variables
        /*ros::Rate r(1);
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
        }*/
    }
    
    void setCurrentTemplate(string templ) {
		currentTemplate = templ;
	}
	
	string getCurrentTemplate() {
		return currentTemplate;
	}
	
	void setCurrentPosition(VectorXd pos) {
		currentPosition = pos;
	}
	
	VectorXd getCurrentPosition() {
		return currentPosition;
	}
};

/*void testCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}*/

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_alignment");
    
    ros::NodeHandle n;
    
    //ros::Subscriber sub = n.subscribe("chatter", 1000, testCallback);

    PointcloudAlignmentAction pointcloud_alignment(ros::this_node::getName());
    ros::spin();

	return 0;
}

std::string get_working_path()
{
	int MAXPATHLEN = 300;
	char temp[MAXPATHLEN];
	return ( getcwd(temp, MAXPATHLEN) ? std::string( temp ) : std::string("") );
}

MatrixXd getTemplatePointcloud(string path, string filename) {
	std::ifstream file;
	string full_filename = path + filename; // TODO: / am Ende von path abfragen
	// TODO: Endung auf .pcd ueberpruefen
	
	cout<<"full filename: "<<full_filename<<endl;
	
	cout<<"cur dir: "<<get_working_path()<<endl;
	
	file.open(full_filename.c_str());
    if (!file.is_open()) {
        std::cerr<<"Error while reading the input file!"<<std::endl;
        return MatrixXd::Identity(3,3);
    }
    
    string tmp;
    do {
		file >> tmp;
	} while (tmp != "POINTS");
	
	int number_points;
	file >> number_points;
	
	file>>tmp;
	file>>tmp;
	
    MatrixXd pointcloud(3,number_points);
    for (int i = 1; i < number_points; i++) {
		file >> pointcloud(0,i);
		file >> pointcloud(1,i);
		file >> pointcloud(2,i);
	} 
	
	return pointcloud;
}

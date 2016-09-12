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

#include <sensor_msgs/PointCloud2.h>
#include <vigir_object_template_msgs/TemplateServerList.h>
#include <vigir_ocs_msgs/OCSObjectSelection.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types_conversion.h>

using namespace Eigen;
using namespace std;

MatrixXd getTemplatePointcloud(string path, string filename);

static int currentTemplateId;
static VectorXd currentPosition;
static MatrixXd currentPointcloud;

class PointcloudAlignmentAction
{
private:


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

};

void templateListCallback(const vigir_object_template_msgs::TemplateServerList::ConstPtr& newTemplateList) {
    //cout<<"I received a new template list"<<endl;
}

void templateSelectionCallback(const vigir_ocs_msgs::OCSObjectSelection::ConstPtr& newTemplate) {
    cout<<"I received a new template"<<endl;
    currentTemplateId = newTemplate->id;
}

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud) {

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pc_ (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*pointcloud, *pc_);


    cout<<"Ich habe 1 Pointcloud empfangen: " << pc_->at(0) <<endl;

    currentPointcloud = MatrixXd(3,pc_->size());

    for (int i = 0; i < pc_->size(); i++) {
        currentPointcloud(0,i) = pc_->at(i).x;
        currentPointcloud(1,i) = pc_->at(i).y;
        currentPointcloud(2,i) = pc_->at(i).z;
    }


}

/*void pointcloudCallback(const object_template_alignment_plugin::PointcloudAlignmentGoalConstPtr &pointcloud) {
    cout<<"Hier ist 1 neue pointcloud"<<endl;
    return;
}*/

/*void testCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}*/

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_alignment");

    ros::NodeHandle n;

    ros::Subscriber sub1 = n.subscribe("/flor/worldmodel/ocs/cloud_result", 1000, pointcloudCallback);

    ros::Subscriber sub2 = n.subscribe("/flor/ocs/object_selection", 1000, templateSelectionCallback);

    ros::Subscriber sub3 = n.subscribe("/template/list", 1000, templateListCallback);

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

#include <ros/ros.h>
#include <ros/package.h>
#include <keyboard/Key.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <icp_alignment_server/PointcloudAlignmentAction.h>
#include <csignal>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <vigir_object_template_msgs/TemplateServerList.h>
#include <vigir_ocs_msgs/OCSObjectSelection.h>

#include <vigir_object_template_msgs/SetAlignObjectTemplate.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
//#include <pcl/
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types_conversion.h>






using namespace std;
using namespace Eigen;

static boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pointmap, scancloud;
static bool pointmapReceived = false, scancloudReceived = false, imuReceived = false;



static int mode = 1;


class PointcloudAlignmentClient
{

protected:
    ros::NodeHandle nh_;
    ros::Subscriber sub1_, sub2_, sub3_;
};

void sendRequestToServer() {

    // check if all necessary information has been received
    if (scancloudReceived == false) {
        ROS_ERROR("No scancloud received - Please send a pointcloud request first.");
        return;
    } else if (pointmapReceived == false) {
        ROS_ERROR("No pointmap received - Please send a pointcloud request first.");
        return;
    } else if (imuReceived == false) {
        ROS_ERROR("No IMU information received - Please send a pointcloud request first.");
        return;
    }

    // contact server
    actionlib::SimpleActionClient<icp_alignment_server::PointcloudAlignmentAction> ac("pointcloud_alignment", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    // convert pointclouds
    static sensor_msgs::PointCloud2 scandata_msg, pointmap_msg;
    pcl::toROSMsg(*scancloud, scandata_msg);
    pcl::toROSMsg(*pointmap, pointmap_msg);

    // create goal and set parameters
    icp_alignment_server::PointcloudAlignmentGoal goal;
    // TODO: ...

    // call server
    ac.sendGoal(goal);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout) {
        icp_alignment_server::PointcloudAlignmentResultConstPtr result = ac.getResult();



        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else {
        ROS_INFO("Action did not finish before the time out.");
    }

    /*
    // create request for action server and wait for server to start
    actionlib::SimpleActionClient<object_template_alignment_server::PointcloudAlignmentAction> ac("pointcloud_alignment", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    // get template name
    std::string filename = currentTemplateName;
    size_t dot_pos = currentTemplateName.find_last_of('.');
    filename.insert(dot_pos, ".pcd");
    filename = filename.substr(0, dot_pos+4);

    // get template location and load template from pcd file
    string path = ros::package::getPath("vigir_template_library") + "/object_library/";
    sensor_msgs::PointCloud2 template_pointcloud;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::io::loadPCDFile(path + filename,*cloud);
    pcl::toROSMsg(*cloud, template_pointcloud);

    // convert target point cloud to msg
    static sensor_msgs::PointCloud2 target_pointcloud;
    pcl::toROSMsg(*world_pointcloud, target_pointcloud);

    // create goal and set parameters
    object_template_alignment_server::PointcloudAlignmentGoal goal;
    goal.source_pointcloud = template_pointcloud;
    goal.target_pointcloud = target_pointcloud;

    geometry_msgs::PoseStamped initial_pose;
    initial_pose.pose.orientation = currentPose.orientation;
    initial_pose.pose.position = currentPose.position;
    goal.initial_pose = initial_pose;

    goal.command = mode;

    // call server
    ac.sendGoal(goal);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout) {
        object_template_alignment_server::PointcloudAlignmentResultConstPtr result = ac.getResult();

        // send current Pose to align_template_srv
        ros::ServiceClient align_template_client;
        vigir_object_template_msgs::SetAlignObjectTemplate align_template_srv_;
        ros::NodeHandle nh_;

        align_template_client = nh_.serviceClient<vigir_object_template_msgs::SetAlignObjectTemplate>("/align_object_template");

        align_template_srv_.request.template_id = currentTemplateId; // toDO: id zwischenspeichern
        align_template_srv_.request.pose.pose.position = result->transformation_pose.pose.position;
        align_template_srv_.request.pose.pose.orientation = result->transformation_pose.pose.orientation;
        align_template_srv_.request.pose.header.stamp = ros::Time::now();
        if (!align_template_client.call(align_template_srv_)) {
            ROS_ERROR("Failed to call service request align template");
        }

        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else {
        ROS_INFO("Action did not finish before the time out.");
    }*/
}

void keyboardCallback(const keyboard::Key::ConstPtr& key) {
    // set command according to input
    int command;
    if (key->code == 108) { // code == l
        mode = 0;
        ROS_INFO("Mode set to local pointcloud alignment.");
    } else if (key->code == 103) { // code == g;
        mode = 1;
        ROS_INFO("Mode set to global pointcloud alignment.");
    }
}

void pointmapCallback(const sensor_msgs::PointCloud2::ConstPtr& pointmap_msg) {
    cout<<"I received a new pointmap" <<endl;

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pc_ (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*pointmap_msg, *pc_);

    pointmap = pc_;

    pointmapReceived = true;
}

void scancloudCallback(const sensor_msgs::PointCloud2::ConstPtr& scancloud_msg) {
    cout<<"I received a new scancloud" <<endl;

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pc_ (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*scancloud_msg, *pc_);

    scancloud = pc_;

    scancloudReceived = true;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {

    imuReceived = true;
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "test_object_template_alignment_plugin");

    ros::NodeHandle nh;
    ros::Subscriber sub1, sub2, sub3, sub4;


    sub1 = nh.subscribe("/point_map", 1, pointmapCallback);

    sub2 = nh.subscribe("/scan_cloud_aggregated", 1, scancloudCallback);

    sub4 = nh.subscribe("/keyboard/keyup", 1, keyboardCallback);

    ros::spin();

    return 0;
}

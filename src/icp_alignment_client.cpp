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
static geometry_msgs::Point validRotationAxis;


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
        ROS_ERROR("No IMU information received - Using z axis as valid rotation axis.");
        validRotationAxis.x = 0;
        validRotationAxis.y = 0;
        validRotationAxis.z = 1;
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
    goal.scancloud = scandata_msg;
    goal.pointmap = pointmap_msg;
    goal.valid_rotation_axis = validRotationAxis;

    // call server
    ac.sendGoal(goal);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout) {
        icp_alignment_server::PointcloudAlignmentResultConstPtr result = ac.getResult();

        // TODO: what should be done with the result?

        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else {
        ROS_INFO("Action did not finish before the time out.");
    }
}

void keyboardCallback(const keyboard::Key::ConstPtr& key) {
    // set command according to input
    int command;
    if (key->code == 97) { // code == a
        ROS_INFO("Sending server request.");
        sendRequestToServer();
    } else {
        ROS_INFO("Key %d pressed", key->code);
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
    cout<<"IMU information received."<<endl;

    //TODO: fill and uncomment in order to use imu information
    /*validRotationAxis(0) = ...
    validRotationAxis(1) = ...
    validRotationAxis(2) = ...
    imuReceived = true;*/
    // Note: If imuReceived remains false, (0,0,1) is used as validRotationAxis (see sendServerRequest())
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "icp_alignment_plugin");

    ros::NodeHandle nh;
    ros::Subscriber sub1, sub2, sub3, sub4;


    sub1 = nh.subscribe("/point_map", 1, pointmapCallback);

    sub2 = nh.subscribe("/scan_cloud_aggregated", 1, scancloudCallback);

    sub3 = nh.subscribe("/imu/data", 1, imuCallback);

    sub4 = nh.subscribe("/keyboard/keyup", 1, keyboardCallback);

    ros::spin();

    return 0;
}

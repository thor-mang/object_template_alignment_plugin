#include <ros/ros.h>
#include <ros/package.h>
#include <keyboard/Key.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_template_alignment_server/PointcloudAlignmentAction.h>
#include <csignal>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

#include <sensor_msgs/PointCloud2.h>
#include <vigir_object_template_msgs/TemplateServerList.h>
#include <vigir_ocs_msgs/OCSObjectSelection.h>

#include <vigir_object_template_msgs/SetAlignObjectTemplate.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types_conversion.h>

#include <time.h>
#include <sys/time.h>

#include <fstream>

using namespace std;
using namespace Eigen;

typedef struct timeval timeval;

void saveData(string path);
void run_test(string test_name, string path, MatrixXf template_cloud, MatrixXf target_cloud, geometry_msgs::PoseStamped pose, float t_err, int n_tests);
bool readInArguments(string path, MatrixXf &source_cloud, MatrixXf &target_cloud, VectorXf &t, VectorXf &q);
void sendServerRequest(MatrixXf source_cloud, MatrixXf target_cloud, VectorXf t, VectorXf R, float &aligned_percentage, float &normalized_error);
void runTests(string test_name, MatrixXf source_cloud, MatrixXf target_cloud, VectorXf t, VectorXf q);
float convertTimeval(timeval t);
VectorXf calcOffset(float dist);
float getRandomNumber();
void updateVals(float val, float &min, float &max);

static int currentTemplateId;
static bool pointcloudReceived = false, templateReceived = false;
static geometry_msgs::PoseStamped currentPose;
static MatrixXf currentWorld, currentTemplate;

class PointcloudAlignmentClient
{

protected:
    ros::NodeHandle nh_;
};

float convertTimeval(timeval t) {
    return (float) ((1.0/1000)*((t.tv_sec * 1000000 + t.tv_usec)));
}

void sendServerRequest(MatrixXf source_cloud, MatrixXf target_cloud, VectorXf t, VectorXf R, float &aligned_percentage, float &normalized_error, float &passed_time) {
    timeval start, end;

    // create initial pose
    geometry_msgs::PoseStamped initialPose;

    geometry_msgs::Quaternion orientation;
    geometry_msgs::Point position;

    position.x = t(0);
    position.y = t(1);
    position.z = t(2);

    orientation.w = sqrt(1. + R(0,0) + R(1,1) + R(2,2)) / 2.;
    orientation.x = (R(2,1) - R(1,2)) / (4.*orientation.w);
    orientation.y = (R(0,2) - R(2,0)) / (4.*orientation.w);
    orientation.z = (R(1,0) - R(0,1)) / (4.*orientation.w);

    initialPose.pose.orientation = orientation;
    initialPose.pose.position = position;

    // convert source cloud to message
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_tmp (new pcl::PointCloud<pcl::PointXYZ>);
    source_tmp->width = source_cloud.cols();
    source_tmp->height = 1;
    source_tmp->points.resize (source_tmp->width * source_tmp->height);

    for (int i = 0; i < source_cloud.cols(); i++) {
        source_tmp->points[i].x = source_cloud(0,i);
        source_tmp->points[i].y = source_cloud(1,i);
        source_tmp->points[i].z = source_cloud(2,i);
    }

    static sensor_msgs::PointCloud2 source_msg;
    pcl::toROSMsg(*source_tmp, source_msg);

    // convert target cloud to message
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_tmp (new pcl::PointCloud<pcl::PointXYZ>);
    target_tmp->width = target_cloud.cols();
    target_tmp->height = 1;
    target_tmp->points.resize (target_tmp->width * target_tmp->height);

    for (int i = 0; i < target_cloud.cols(); i++) {
        target_tmp->points[i].x = target_cloud(0,i);
        target_tmp->points[i].y = target_cloud(1,i);
        target_tmp->points[i].z = target_cloud(2,i);
    }

    static sensor_msgs::PointCloud2 target_msg;
    pcl::toROSMsg(*target_tmp, target_msg);

    // send goal to server
    actionlib::SimpleActionClient<object_template_alignment_server::PointcloudAlignmentAction> ac("pointcloud_alignment", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    object_template_alignment_server::PointcloudAlignmentGoal goal;
    goal.initial_pose = initialPose;
    goal.source_pointcloud = source_msg;
    goal.target_pointcloud = target_msg;
    goal.command = 1;

    gettimeofday(&start, NULL);
    ac.sendGoal(goal);
    gettimeofday(&end, NULL);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout) {
        object_template_alignment_server::PointcloudAlignmentResultConstPtr result = ac.getResult();

        aligned_percentage = result->aligned_percentage;
        normalized_error = result->normalized_error;
        passed_time = convertTimeval(end)-convertTimeval(start);
    }
}

void runTests(string filename, int n_tests, float offset, MatrixXf source_cloud, MatrixXf target_cloud, VectorXf t, MatrixXf R) {
    float aligned_percentage, normalized_error, passed_time;
    float min_percentage = FLT_MAX, max_percentage = FLT_MIN, avg_percentage = 0;
    float min_error = FLT_MAX, max_error = FLT_MIN, avg_error = 0;
    float min_time = FLT_MAX, max_time = FLT_MIN, avg_time = 0;

    for (int i = 0; i < n_tests; i++) {
        // TODO: aufruf überarbeiten
        sendServerRequest(source_cloud, target_cloud, t, R, aligned_percentage, normalized_error, passed_time);

        avg_percentage += aligned_percentage;
        updateVals(aligned_percentage, min_percentage, max_percentage);

        avg_error += normalized_error;
        updateVals(normalized_error, min_error, max_error);

        avg_time += passed_time;
        updateVals(passed_time, min_time, max_time);
    }

    avg_percentage /= ((float) n_tests);
    avg_error /= ((float) n_tests);
    avg_time /= ((float) n_tests);

    std::ofstream file;
    file.open(filename.c_str(), std::ios::app);
    file << "\t avg_percentage: "<<avg_percentage<<", min_percentage: "<<min_percentage<<", max_percentage: "<<max_percentage<<endl;
    file << "\t avg_error: "<<avg_error<<", min_error: "<<min_error<<", max_error: "<<max_error<<endl;
    file << "\t avg_time: "<<avg_time<<", min_time: "<<min_time<<", max_time: "<<max_time<<endl;
    file.close();
}

void updateVals(float val, float &min, float &max) {
    if (val < min) {
        min = val;
    }
    if (val > max) {
        max = val;
    }
}

VectorXf calcOffset(float dist) {
    float alpha = getRandomNumber() * M_PI;
    float beta = getRandomNumber() * 2.*M_PI;

    VectorXf offset(3);
    offset(0) = dist*sin(alpha)*cos(beta);
    offset(1) = dist*sin(alpha)*sin(beta);
    offset(2) = dist*cos(alpha);
}

float getRandomNumber() {
    return ((float) rand() / ((float ) RAND_MAX));
}

void traverse_directories(string test_data_dir) {
    DIR *dir, *sub_dir;
    struct dirent *entry, *sub_entry;
    if ((dir = opendir (test_data_dir.c_str())) != NULL) {
        while ((entry = readdir (dir)) != NULL) {
            if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
                continue;
            }

            if (entry->d_type == DT_DIR) {
                string sub_dir_name = test_data_dir + "/" + entry->d_name;
                if ((sub_dir = opendir(sub_dir_name.c_str())) != NULL) {

                    VectorXf t, q;
                    MatrixXf source_cloud, target_cloud;

                    if (readInArguments(sub_dir_name, source_cloud, target_cloud, t, q) == false) {
                        ROS_ERROR("%s contains not all data!", sub_dir_name.c_str());
                        return;
                    }

                    MatrixXf R(3,3);
                    R <<
                             1.0f - 2.0f*q(1)*q(1) - 2.0f*q(2)*q(2), 2.0f*q(0)*q(1) - 2.0f*q(2)*q(3), 2.0f*q(0)*q(2) + 2.0f*q(1)*q(3),
                             2.0f*q(0)*q(1) + 2.0f*q(2)*q(3), 1.0f - 2.0f*q(0)*q(0) - 2.0f*q(2)*q(2), 2.0f*q(1)*q(2) - 2.0f*q(0)*q(3),
                             2.0f*q(0)*q(2) - 2.0f*q(1)*q(3), 2.0f*q(1)*q(2) + 2.0f*q(0)*q(3), 1.0f - 2.0f*q(0)*q(0) - 2.0f*q(1)*q(1);

                    //runTests(sub_dir_name, source_cloud, target_cloud, t, R);
                    string filename = sub_dir_name + ".txt";
                    cout<<"name: "<<filename<<endl;
                    std::ofstream file;
                    file.open(filename.c_str());
                    file << "running test 1 with 100 iterations"<<endl;
                    file.close();

                    runTests(filename, 0, 0, source_cloud, target_cloud, t, R);
                }
            }
        }
        closedir (dir);
    } else {
        ROS_ERROR("Could not open test_data_dir");
    }
}

bool readInArguments(string path, MatrixXf &source_cloud, MatrixXf &target_cloud, VectorXf &t, VectorXf &q) {
    ifstream file;
    int cols;

    // read in position
    t = VectorXf(3);
    string filename = path + "/position.txt";
    file.open(filename.c_str());
    if (!file.is_open()) {
        ROS_ERROR("Failed to open %s", filename.c_str());
        return false;
    }
    for (int i = 0; i < 3; i++) {
        file >> t(i);
    }
    file.close();

    // read in orientation
    q = VectorXf(4);
    filename = path + "/orientation.txt";
    file.open(filename.c_str());
    if (!file.is_open()) {
        ROS_ERROR("Failed to open %s", filename.c_str());
        return false;
    }
    for (int i = 0; i < 4; i++) {
        file >> q(i);
    }
    file.close();

    // read in source cloud
    filename = path + "/source.txt";
    file.open(filename.c_str());
    if (!file.is_open()) {
        ROS_ERROR("Failed to open %s", filename.c_str());
        return false;
    }
    file >> cols;
    source_cloud = MatrixXf(3,cols);
    for (int i = 0; i < cols; i++) {
        file >> source_cloud(0,i);
        file >> source_cloud(1,i);
        file >> source_cloud(2,i);
    }
    file.close();

    // read in target cloud
    filename = path + "/target.txt";
    file.open(filename.c_str());
    if (!file.is_open()) {
        ROS_ERROR("Failed to open %s", filename.c_str());
        return false;
    }
    file >> cols;
    target_cloud = MatrixXf(3,cols);
    for (int i = 0; i < cols; i++) {
        file >> target_cloud(0,i);
        file >> target_cloud(1,i);
        file >> target_cloud(2,i);
    }
    file.close();

    return true;
}

void saveData(string path) {
    if (pointcloudReceived == false) {
        ROS_ERROR("Pointcloud missing!");
        return;
    }

    ofstream file;
    string filename;

    // save position
    filename = path + "/position.txt";
    file.open(filename.c_str());
    file << currentPose.pose.position.x << " "<< currentPose.pose.position.y << " "<< currentPose.pose.position.z << endl;
    file.close();

    // save orientation
    filename = path + "/orientation.txt";
    file.open(filename.c_str());
    file << currentPose.pose.orientation.x << " "<< currentPose.pose.orientation.y << " "<< currentPose.pose.orientation.z << " "<< currentPose.pose.orientation.w<<endl;
    file.close();

    // save world cloud
    filename = path + "/target.txt";
    file.open(filename.c_str());
    file << currentWorld.cols()<<endl;
    for (int i = 0; i < currentWorld.cols(); i++) {
        file<<currentWorld(0,i)<<" " << currentWorld(1,i)<<" "<<currentWorld(2,i)<<endl;
    }
    file.close();

    // save template
    filename = path + "/source.txt";
    file.open(filename.c_str());
    file << currentTemplate.cols()<<endl;
    for (int i = 0; i < currentTemplate.cols(); i++) {
        file<<currentTemplate(0,i)<<" " << currentTemplate(1,i)<<" "<<currentTemplate(2,i)<<endl;
    }
    file.close();
}

void keyboardCallback(const keyboard::Key::ConstPtr& key) {
    // set command according to input
    int command;
    if (key->code == 116) { // code == t
        string path = ros::package::getPath("object_template_alignment_plugin") + "/test_data/";
        traverse_directories(path);
    } else {
        ROS_INFO("key %d pressed", key->code);
    }
}

MatrixXf readInTemplate(string template_name) {
    std::string filename = template_name;
    size_t dot_pos = template_name.find_last_of('.');
    filename.insert(dot_pos, ".pcd");
    filename = filename.substr(0, dot_pos+4);

    string path = ros::package::getPath("vigir_template_library") + "/object_library/";
    sensor_msgs::PointCloud2 template_pointcloud;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud (new pcl::PointCloud<pcl::PointXYZ>());
    string full_filename = path + filename;
    pcl::io::loadPCDFile(full_filename,*cloud); // TODO: Fehler abfangen
    pcl::toROSMsg(*cloud, template_pointcloud);

    currentTemplate = MatrixXf(3, cloud->size());
    for (int i = 0; i < cloud->size(); i++) {
        currentTemplate(0,i) = cloud->at(i).x;
        currentTemplate(1,i) = cloud->at(i).y;
        currentTemplate(2,i) = cloud->at(i).z;
    }

    return currentTemplate;
}

void templateListCallback(const vigir_object_template_msgs::TemplateServerList::ConstPtr& templateList) {
    if (templateReceived == true) {
        templateReceived = false;

        // get position of the current template in the template list
        int pos = -1;
        for (int i = 0; i < templateList->template_id_list.size(); i++) {

            if (templateList->template_id_list.at(i) == currentTemplateId) {
                pos = i;
                break;
            }
        }

        if (pos != -1) {
            // read out current pose and name of the template
            string tn = templateList->template_list.at(pos);
            currentTemplate = readInTemplate(tn);
            currentPose = templateList->pose.at(pos);

            cout<<"saving data ..."<<endl;
            saveData("/home/sebastian/thor/src/object_template_alignment_plugin/test_data/");
        }
    }
}

void templateSelectionCallback(const vigir_ocs_msgs::OCSObjectSelection::ConstPtr& newTemplate) {
    cout<<"I received a new template"<<endl;

    currentTemplateId = newTemplate->id;


    templateReceived = true;
}

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud) {
    cout<<"I received a new pointcloud" <<endl;

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pc_ (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*pointcloud, *pc_);

    currentWorld = MatrixXf(3,pc_->size());

    for (int i = 0; i < pc_->size(); i++) {
        currentWorld(0,i) = pc_->at(i).x;
        currentWorld(1,i) = pc_->at(i).y;
        currentWorld(2,i) = pc_->at(i).z;
    }

    pointcloudReceived = true;
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "test_object_template_alignment_plugin");

    ros::NodeHandle nh;
    ros::Subscriber sub1, sub2, sub3, sub4;

    sub1 = nh.subscribe("/flor/worldmodel/ocs/cloud_result", 1, pointcloudCallback);

    sub2 = nh.subscribe("/flor/ocs/object_selection", 1, templateSelectionCallback);

    sub3 = nh.subscribe("/template/list", 100, templateListCallback);

    sub4 = nh.subscribe("/keyboard/keyup", 1, keyboardCallback);

    ros::spin();

    return 0;
}

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
void sendServerRequest(string test_data_filename, MatrixXf source_cloud, MatrixXf target_cloud, VectorXf t, VectorXf R, float &aligned_percentage, float &normalized_error, float &passed_time);
void runTests(string filename, int n_tests, float offset, bool dense, MatrixXf source_cloud, MatrixXf target_cloud, VectorXf t, MatrixXf R);
float convertTimeval(timeval start, timeval end);
VectorXf calcOffset(float dist, bool dense);
float getRandomNumber();
void updateVals(float val, float &min, float &max);
MatrixXf randomRotation();
MatrixXf quaternionToMatrix(VectorXf q);
void increaseTestCounter();
void save_test_data(string filename, float normalized_error, float overlapping_percentage, VectorXf pos, VectorXf q, float passed_time);
void initCSVfile(string filename, string first_entry);
void writeToCSVfile(string filename, float distance, float success_rate, float avg_time, float min_time, float max_time, float avg_error, float min_error, float max_error);
void writeToCSVfile(string filename, string test_name, float success_rate, float avg_time, float min_time, float max_time, float avg_error, float min_error, float max_error);
float calcMaxDistance(MatrixXf const &pc);

static int currentTemplateId;
static bool pointcloudReceived = false, templateReceived = false;
static geometry_msgs::PoseStamped currentPose;
static MatrixXf currentWorld, currentTemplate;

static int global_test_counter = 0, global_number_tests = 15600; // TODO!!

static float percentage_save = 0, error_save = FLT_MAX;

class PointcloudAlignmentClient
{

protected:
    ros::NodeHandle nh_;
};

void doneCb(const actionlib::SimpleClientGoalState& state, const object_template_alignment_server::PointcloudAlignmentResultConstPtr& result) {}

void activeCb() {}

void feedbackCb(const object_template_alignment_server::PointcloudAlignmentFeedbackConstPtr& feedback) {
    percentage_save = feedback->aligned_percentage;
    error_save = feedback->normalized_error;
}

float convertTimeval(timeval start, timeval end) {
    //return (float) ((((end.tv_sec * 1000000 + end.tv_usec) - (start.tv_sec * 1000000 + start.tv_usec)))/1000.);
    return ((float) (((end.tv_sec - start.tv_sec)*1000000L + end.tv_usec) - start.tv_usec)) / 1000000.;
}

void sendServerRequest(string test_data_filename, MatrixXf source_cloud, MatrixXf target_cloud, VectorXf &t, MatrixXf &R, float &aligned_percentage, float &normalized_error, float &passed_time) {
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
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);


    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout) {
        object_template_alignment_server::PointcloudAlignmentResultConstPtr result = ac.getResult();
        gettimeofday(&end, NULL);

        aligned_percentage = percentage_save;
        normalized_error = error_save;
        passed_time = convertTimeval(start, end);

        t<< result->transformation_pose.pose.position.x, result->transformation_pose.pose.position.y, result->transformation_pose.pose.position.z;
        VectorXf q(4);
        q << result->transformation_pose.pose.orientation.x,result->transformation_pose.pose.orientation.y,
             result->transformation_pose.pose.orientation.z,result->transformation_pose.pose.orientation.w;
        R = quaternionToMatrix(q);

        //save_test_data(test_data_filename, error_save, percentage_save, t, q, passed_time);
    }
}

void runTests(string test_name, int n_tests, float dist, bool regular_test, MatrixXf source_cloud, MatrixXf target_cloud, VectorXf t, MatrixXf R) {
    float aligned_percentage, normalized_error, passed_time;
    float min_percentage = FLT_MAX, max_percentage = FLT_MIN, avg_percentage = 0;
    float min_error = FLT_MAX, max_error = FLT_MIN, avg_error = 0;
    float min_time = FLT_MAX, max_time = FLT_MIN, avg_time = 0;

    float MAX_ROTATION_ERROR = 0.1;
    float MAX_TRANSLATION_ERROR = 0.2;

    int success_ct = 0;
    float success_rate;

    string test_data_filename = test_name.substr(0, test_name.length()-4) + "_test_data.txt";

    for (int i = 0; i < n_tests; i++) {
        increaseTestCounter();

        VectorXf t_pa = t + calcOffset(dist, regular_test);
        MatrixXf R_pa = randomRotation();

        sendServerRequest(test_data_filename, source_cloud, target_cloud, t_pa, R_pa, aligned_percentage, normalized_error, passed_time);


        if ((t-t_pa).norm() < MAX_TRANSLATION_ERROR && (R-R_pa).norm() < MAX_ROTATION_ERROR) {
            success_ct++;
        }

        avg_percentage += aligned_percentage;
        updateVals(aligned_percentage, min_percentage, max_percentage);

        avg_error += normalized_error;
        updateVals(normalized_error, min_error, max_error);

        avg_time += passed_time;
        updateVals(passed_time, min_time, max_time);
    }


    success_rate = (((float) success_ct) / ((float) n_tests));
    avg_percentage /= ((float) n_tests);
    avg_error /= ((float) n_tests);
    avg_time /= ((float) n_tests);

    cout<<"hier "<<regular_test<<endl;

    if (regular_test == true) {
        cout<<"now writing regular test to CSV file"<<endl;
        writeToCSVfile("/home/sebastian/thor/src/object_template_alignment_plugin/test_data/regular_tests", test_name, success_rate, avg_time, min_time, max_time, avg_error, min_error, max_error);
    } else {
        cout<<"now writing distance test to CSV file"<<endl;
        writeToCSVfile(test_name, dist, success_rate, avg_time, min_time, max_time, avg_error, min_error, max_error);
    }
}

void save_test_data(string filename, float normalized_error, float overlapping_percentage, VectorXf pos, VectorXf q, float passed_time) {
    std::ofstream file;
    file.open(filename.c_str(), std::ios::app);

    file << "passed_time: "<<passed_time<<endl;
    file << "normalized_error: "<<normalized_error<<endl;
    file << "overlapping_percentage: "<<overlapping_percentage<<endl;
    file << "position: "<<pos(0)<<" "<<pos(1)<<" "<<pos(2)<<endl;
    file << "orientation: "<<q(0)<<" "<<q(1)<<" "<<q(2)<<" "<<q(3)<<endl;
    file << endl;

    file.close();
}

void traverse_directories(string test_data_dir) {
    DIR *dir, *sub_dir;
    struct dirent *entry;
    if ((dir = opendir (test_data_dir.c_str())) != NULL) {
        while ((entry = readdir (dir)) != NULL) {
            if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
                continue;
            }

            if (entry->d_type == DT_DIR) {
                string sub_dir_name = test_data_dir + entry->d_name;
                if ((sub_dir = opendir(sub_dir_name.c_str())) != NULL) {

                    VectorXf t, q;
                    MatrixXf source_cloud, target_cloud;

                    if (readInArguments(sub_dir_name, source_cloud, target_cloud, t, q) == false) {
                        ROS_ERROR("%s contains not all data!", sub_dir_name.c_str());
                        return;
                    }

                    MatrixXf R = quaternionToMatrix(q);

                    string test_name = entry->d_name;

                    int n_distance_tests = 2, n_regular_tests = 2;
                    float distance_factor = 1.;


                    // execute distance tests
                    if (test_name.substr(0,15).compare("drill_on_table_") == 0) {
                        cout<<"drill_on_table"<<endl;
                        string filename = sub_dir_name + "_distances_results";

                        initCSVfile(filename, "distance");
                        for (int i = 0; i <= 20; i++) {
                            float dist = ((float) i) * 0.02;

                            runTests(filename, n_distance_tests, dist, false, source_cloud, target_cloud, t, R);
                        } 
                    }

                    // execute regular tests

                    /*initCSVfile("/home/sebastian/thor/src/object_template_alignment_plugin/test_data/regular_tests", "test_name");

                    float maxDist = calcMaxDistance(source_cloud);

                    float dist = maxDist*distance_factor;

                    runTests(test_name, n_regular_tests, dist, true, source_cloud, target_cloud, t, R);*/
                }
            }
        }

        closedir (dir);
    } else {
        ROS_ERROR("Could not open test_data_dir");
    }
}

float calcMaxDistance(MatrixXf const &pc) {
    VectorXf midPoint(3);
    midPoint<<0,0,0;
    for (int i = 0; i < pc.cols(); i++) {
        midPoint(0) += pc(0,i);
        midPoint(1) += pc(1,i);
        midPoint(2) += pc(2,i);
    }
    midPoint = midPoint / ((float) pc.cols());

    float maxDist = FLT_MIN;
    float dist;
    VectorXf distVec(3);
    for (int i = 0; i < pc.cols(); i++) {
        distVec(0) = pc(0,i) - midPoint(0);
        distVec(1) = pc(1,i) - midPoint(1);
        distVec(2) = pc(2,i) - midPoint(2);

        dist = sqrt(distVec(0)*distVec(0) + distVec(1)*distVec(1) + distVec(2)*distVec(2));

        if (dist > maxDist) {
            maxDist = dist;
        }
    }

    return maxDist;
}

void initCSVfile(string filename, string first_entry) {
    cout<<"initCSVfile: "<<filename<<endl;
    filename = filename + ".csv";
    std::ofstream file;
    file.open(filename.c_str());

    file << first_entry << ", success_rate, avg_time, min_time, max_time, avg_error, min_error, max_error" << endl;

    file.close();
}

void writeToCSVfile(string filename, float distance, float success_rate, float avg_time, float min_time, float max_time, float avg_error, float min_error, float max_error) {
    cout<<"writeToCSVfile: "<<filename<<endl;
    filename = filename + ".csv";

    std::ofstream file;
    file.open(filename.c_str(), std::ios::app);

    file << distance <<","<< success_rate <<","<< avg_time <<","<< min_time <<","<< max_time <<","<< avg_error <<","<< min_error <<","<< max_error << endl;

    file.close();
}

void writeToCSVfile(string filename, string test_name, float success_rate, float avg_time, float min_time, float max_time, float avg_error, float min_error, float max_error) {
    cout<<"writeToCSVfile: "<<filename<<" "<<test_name<<endl;
    filename = filename + ".csv";
    std::ofstream file;
    file.open(filename.c_str(), std::ios::app);

    file << test_name <<","<< success_rate <<","<< avg_time <<","<< min_time <<","<< max_time <<","<< avg_error <<","<< min_error <<","<< max_error << endl;

    file.close();
}

void increaseTestCounter() {
    global_test_counter++;

    float average_test_time = 2.5;

    int remaining_tests = global_number_tests - global_test_counter;
    float remaining_time = (((float) remaining_tests) * average_test_time) / 3600.;
    int remaining_h = (int) remaining_time;
    int remaining_min = (((int)((remaining_time-((float)remaining_h)) * 100)) * 60) / 100;
    ROS_INFO(" ");
    ROS_INFO("Executing test %d/%d...", global_test_counter, global_number_tests);
    ROS_INFO("Estimated remaining time: %dh %d m", remaining_h, remaining_min);

}

void updateVals(float val, float &min, float &max) {
    if (val < min) {
        min = val;
    }
    if (val > max) {
        max = val;
    }
}

MatrixXf quaternionToMatrix(VectorXf q) {
    MatrixXf R(3,3);
    R <<
             1.0f - 2.0f*q(1)*q(1) - 2.0f*q(2)*q(2), 2.0f*q(0)*q(1) - 2.0f*q(2)*q(3), 2.0f*q(0)*q(2) + 2.0f*q(1)*q(3),
             2.0f*q(0)*q(1) + 2.0f*q(2)*q(3), 1.0f - 2.0f*q(0)*q(0) - 2.0f*q(2)*q(2), 2.0f*q(1)*q(2) - 2.0f*q(0)*q(3),
             2.0f*q(0)*q(2) - 2.0f*q(1)*q(3), 2.0f*q(1)*q(2) + 2.0f*q(0)*q(3), 1.0f - 2.0f*q(0)*q(0) - 2.0f*q(1)*q(1);

    return R;
}

MatrixXf randomRotation() {
    float alpha = getRandomNumber() * M_PI;
    float beta = getRandomNumber() * 2.*M_PI;
    float dist = getRandomNumber()*M_PI;

    VectorXf r(3);
    r(0) = dist*sin(alpha)*cos(beta);
    r(1) = dist*sin(alpha)*sin(beta);
    r(2) = dist*cos(alpha);

    MatrixXf R = MatrixXf::Identity(3,3);

    if (r.norm() == 0) {
        return R;
    }

    MatrixXf r_x(3,3);
    r_x << 0, -r(2), r(1),
        r(2), 0, -r(0),
        -r(1), r(0), 0;


    R += (r_x*sin(r.norm()))/(r.norm());
    R += (r_x*r_x*(1-cos(r.norm())))/(r.norm()*r.norm());

    return R;
}

VectorXf calcOffset(float dist, bool dense) {
    float alpha = getRandomNumber() * M_PI;
    float beta = getRandomNumber() * 2.*M_PI;

    if (dense == true) {
        dist = getRandomNumber() * dist;
    }

    VectorXf offset(3);
    offset(0) = dist*sin(alpha)*cos(beta);
    offset(1) = dist*sin(alpha)*sin(beta);
    offset(2) = dist*cos(alpha);


    return offset;
}

float getRandomNumber() {
    return ((float) rand() / ((float ) RAND_MAX));
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
    ros::Subscriber sub1, sub2, sub3, sub4, sub5;

    sub1 = nh.subscribe("/flor/worldmodel/ocs/cloud_result", 1, pointcloudCallback);

    sub2 = nh.subscribe("/flor/ocs/object_selection", 1, templateSelectionCallback);

    sub3 = nh.subscribe("/template/list", 100, templateListCallback);

    sub4 = nh.subscribe("/keyboard/keyup", 1, keyboardCallback);

    //sub5 = nh.subscribe("/pointcloud_alignment/feedback",1, feedbackCb);

    ros::spin();

    return 0;
}

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

#include <fstream>

using namespace std;
using namespace Eigen;

MatrixXf readInMatrix(string filename, int rows, int cols);
VectorXf readInVector(string filename, int rows);
void saveData(string path);
MatrixXf readInMatrix(string filename);
string readInString(string filename);
void run_test(string test_name, string path, MatrixXf template_cloud, MatrixXf target_cloud, geometry_msgs::PoseStamped pose, float t_err, int n_tests);


static int currentTemplateId;

static bool pointcloudReceived = false, templateReceived = false;

static geometry_msgs::PoseStamped currentPose;
static MatrixXf currentWorld, currentTemplate;

// for runTests



class PointcloudAlignmentClient
{

protected:
    ros::NodeHandle nh_;
};

void sendServerRequest(MatrixXf template_cloud, MatrixXf world_cloud, geometry_msgs::PoseStamped initialPose) {

}

void readInPosition(string filename, geometry_msgs::PoseStamped &pose) {
    ifstream file;
    file.open(filename.c_str());

    file >> pose.pose.position.x;
    file >> pose.pose.position.y;
    file >> pose.pose.position.z;

    file.close();
}

void readInOrientation(string filename, geometry_msgs::PoseStamped &pose) {
    ifstream file;
    file.open(filename.c_str());

    file >> pose.pose.orientation.x;
    file >> pose.pose.orientation.y;
    file >> pose.pose.orientation.z;
    file >> pose.pose.orientation.w;

    file.close();
}

void run_tests(string test_data_dir) {
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

                    MatrixXf template_cloud, world_cloud;
                    geometry_msgs::PoseStamped initialPose;

                    bool world_b = false, template_b = false, rotation_b = false, translation_b = false;

                    while ((sub_entry = readdir (sub_dir)) != NULL) {
                        if (strcmp(sub_entry->d_name, ".") == 0 || strcmp(sub_entry->d_name, "..") == 0) {
                            continue;
                        }

                        if (strcmp(sub_entry->d_name,"orientation.txt") == 0) {
                            string filename = sub_dir_name + "/orientation.txt";
                            readInOrientation(filename, initialPose);
                            rotation_b = true;
                        } else if (strcmp(sub_entry->d_name,"position.txt") == 0) {
                            string filename = sub_dir_name + "/position.txt";
                            readInPosition(filename, initialPose);
                            translation_b = true;
                        } else if (strcmp(sub_entry->d_name,"world.txt") == 0) {
                            string filename = sub_dir_name + "/world.txt";
                            world_cloud = readInMatrix(filename);
                            world_b = true;
                        } else if (strcmp(sub_entry->d_name,"template.txt") == 0) {
                            string filename = sub_dir_name + "/template.txt";
                            template_cloud = readInMatrix(filename);
                            template_b = true;
                        }

                        if (world_b == false || template_b == false || rotation_b == false || template_b == false) {
                            ROS_ERROR("%s contains not all data!", sub_dir_name.c_str());
                            return;
                        }

                        run_test(sub_dir_name, test_data_dir, template_cloud, world_cloud, initialPose, 0, 1);
                    }
                } else {
                    ROS_ERROR("Could not open dir %s", entry->d_name);
                }
            }
        }
        closedir (dir);
    } else {
        ROS_ERROR("Could not open test_data_dir");
    }
}

void saveData(string path) {
    if (templateReceived == false || pointcloudReceived == false) {
        ROS_ERROR("Pointcloud or template missing!");
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
    filename = path + "/world.txt";
    file.open(filename.c_str());
    for (int i = 0; i < currentWorld.cols(); i++) {
        file<<currentWorld(0,i)<<" " << currentWorld(1,i)<<" "<<currentWorld(2,i)<<endl;
    }
    file.close();

    // save template
    filename = path + "/template.txt";
    file.open(filename.c_str());
    for (int i = 0; i < currentTemplate.cols(); i++) {
        file<<currentTemplate(0,i)<<" " << currentTemplate(1,i)<<" "<<currentTemplate(2,i)<<endl;
    }
    file.close();
}

void run_test(string test_name, string path, MatrixXf template_cloud, MatrixXf target_cloud, geometry_msgs::PoseStamped pose, float t_err, int n_tests) {
    for (int i = 0; i < n_tests; i++) {

    }
}

VectorXf readInVector(string filename, int rows) {
    VectorXf vec(rows);

    std::ifstream file;

    file.open(filename.c_str());
    if (!file.is_open()) {
        std::cout<<"Error while reading the input file! "<< filename<<" does not exist or is corrupted!"<<std::endl;
    }

    for (int i = 0; i < rows; i++) {
        file >> vec(i);
    }

    return vec;
}

MatrixXf readInMatrix(string filename, int rows, int cols) {
    MatrixXf mat(rows, cols);

    std::ifstream file;

    file.open(filename.c_str());
    if (!file.is_open()) {
        std::cout<<"Error while reading the input file! "<< filename<<" does not exist or is corrupted!"<<std::endl;
    }

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            file >> mat(i,j);
        }
    }

    return mat;
}

MatrixXf readInMatrix(string filename) {
    std::ifstream file;

    file.open(filename.c_str());
    if (!file.is_open()) {
        std::cout<<"Error while reading the input file! "<< filename<<" does not exist or is corrupted!"<<std::endl;
    }

    int rows, cols;
    file >> rows;
    file >> cols;

    MatrixXf mat(rows, cols);

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            file >> mat(i,j);
        }
    }

    return mat;
}

void keyboardCallback(const keyboard::Key::ConstPtr& key) {
    // set command according to input
    int command;
    if (key->code == 116) { // code == t
        string path = ros::package::getPath("object_template_alignment_plugin") + "/test_data/";
        cout<<"path: "<<path<<endl;
        run_tests(path);
    } else if (key->code == 115) { // code == s;
        saveData("/home/sebastian/Desktop/");
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
    pcl::io::loadPCDFile(path + filename,*cloud); // TODO: Fehler abfangen
    pcl::toROSMsg(*cloud, template_pointcloud);

    currentTemplate = MatrixXf(3, cloud->size());
    for (int i = 0; i < cloud->size(); i++) {
        currentTemplate(0,i) = cloud->at(i).x;
        currentTemplate(1,i) = cloud->at(i).y;
        currentTemplate(2,i) = cloud->at(i).z;
    }
}

void templateListCallback(const vigir_object_template_msgs::TemplateServerList::ConstPtr& templateList) {
    // get position of the current template in the template list
    int pos = -1;
    for (int i = 0; i < templateList->template_id_list.size(); i++) {

        if (templateList->template_id_list.at(i) == currentTemplateId) {
            pos = i;
            break;
        }
    }

    // read out current pose of the template
    if (pos != -1) {
        currentPose = templateList->pose.at(pos);

        currentTemplate = readInTemplate(templateList->template_list.at(pos));
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

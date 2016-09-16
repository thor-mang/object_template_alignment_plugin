#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <object_template_alignment_plugin/PointcloudAlignmentAction.h>
#include <geometry_msgs/PoseStamped.h>

#include <string>
#include <iostream>
#include <stdlib.h>
#include <fstream>

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

#include <pthread.h>
#include <omp.h>
#include <vector>
#include <algorithm>
#include <float.h>

#include <vigir_object_template_msgs/SetAlignObjectTemplate.h>


using namespace Eigen;
using namespace std;

MatrixXf getTemplatePointcloud(string path, string filename);
void saveDataToFile(VectorXf transformation, MatrixXf rotation, MatrixXf template_cloud, MatrixXf world_cloud);

int get_inlier_number(float inlier_portion, int number_points);
float trimmed_scaling_icp(MatrixXf pc1, MatrixXf pc2, MatrixXf &R_result, VectorXf &t_result, float &s_result, MatrixXf R_init,
        VectorXf t_init, float s_init, float eps, int maxIt, float inlier_portion);
void trim_pc(MatrixXf pc, MatrixXf cp, VectorXf distances, MatrixXf &pc_trimmed, MatrixXf &cp_trimmed, int number_inliers);
void find_correspondences(MatrixXf pc1, MatrixXf pc2, MatrixXf &cp, VectorXf &distances);
void find_transformation(MatrixXf pc1, MatrixXf pc2, MatrixXf &R, VectorXf &t, float &s);
void apply_transformation(MatrixXf pc, MatrixXf &pc_proj, MatrixXf R, VectorXf t, float s);
float calc_error(MatrixXf pc1, MatrixXf pc2, float inlier_portion);
float calc_error(MatrixXf pc1, MatrixXf pc2);
void sort(VectorXf &v);
VectorXf matrixToVector(MatrixXf m);

static int currentTemplateId;
static VectorXf currentPosition;
static MatrixXf currentRotation;
static MatrixXf currentTargetPointcloud;

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

        std::string filename = "DRC_drill.pcd";
        std::string path = "vigir/vigir_templates/vigir_template_library/object_library/tools/";

        MatrixXf template_pointcloud = getTemplatePointcloud(path, filename);


        MatrixXf R_icp(3,3);
        VectorXf t_icp(3);
        float s_icp;


        R_icp = MatrixXf::Identity(3,3);
        t_icp << 0,0,0;
        s_icp = 1;


        float error = trimmed_scaling_icp(template_pointcloud, currentTargetPointcloud, R_icp, t_icp, s_icp, currentRotation, currentPosition, 1.0, 1e-3, 100, 0.4);

        cout<<"error: "<<error<<endl;

        //saveDataToFile(currentPosition, currentRotation, template_pointcloud, currentTargetPointcloud);


        if (success)
        {
        geometry_msgs::Quaternion orientation;
        geometry_msgs::Point position;

        position.x = t_icp(0);
        position.y = t_icp(1);
        position.z = t_icp(2);

        orientation.w = sqrt(1. + R_icp(0,0) + R_icp(1,1) + R_icp(2,2)); // TODO: Division durch Null abfangen
        orientation.x = (R_icp(2,1) - R_icp(1,2)) / (4.*orientation.w);
        orientation.y = (R_icp(0,2) - R_icp(2,0)) / (4.*orientation.w);
        orientation.z = (R_icp(1,0) - R_icp(0,1)) / (4.*orientation.w);


        ros::ServiceClient align_template_client;
        vigir_object_template_msgs::SetAlignObjectTemplate align_template_srv_;

        align_template_client = nh_.serviceClient<vigir_object_template_msgs::SetAlignObjectTemplate>("/align_object_template");

        align_template_srv_.request.template_id = currentTemplateId;
        align_template_srv_.request.pose.pose.position = position;
        align_template_srv_.request.pose.pose.orientation = orientation;
        if (!align_template_client.call(align_template_srv_))
        {
           ROS_ERROR("Failed to call service request align template");
           cout<<"align template client success"<<endl;
        } else {
            cout<<"align template client failed"<<endl;
        }

        geometry_msgs::PoseStamped result;

        result.pose.orientation = orientation;
        result.pose.position = position;
        result_.transformation_matrix = result;

        // TODO: header setzen
        result_.transformation_matrix.header.stamp = ros::Time::now();

        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_);
        }
    }

};



void templateListCallback(const vigir_object_template_msgs::TemplateServerList::ConstPtr& templateList) {
    //cout<<"I received a new template list"<<endl;
    int pos = -1;
    for (int i = 0; i < templateList->template_id_list.size(); i++) {

        if (templateList->template_id_list.at(i) == currentTemplateId) {
            pos = i;
            break;
        }
    }

    //templateList->template_status_list.at(0).header;


    if (pos != -1) {
        currentPosition = VectorXf(3);
        currentPosition << templateList->pose.at(pos).pose.position.x,
                           templateList->pose.at(pos).pose.position.y,
                           templateList->pose.at(pos).pose.position.z;

        currentRotation = MatrixXf(3,3);
        float qx = templateList->pose.at(pos).pose.orientation.x;
        float qy = templateList->pose.at(pos).pose.orientation.y;
        float qz = templateList->pose.at(pos).pose.orientation.z;
        float qw = templateList->pose.at(pos).pose.orientation.w;

        currentRotation <<
            1.0f - 2.0f*qy*qy - 2.0f*qz*qz, 2.0f*qx*qy - 2.0f*qz*qw, 2.0f*qx*qz + 2.0f*qy*qw,
            2.0f*qx*qy + 2.0f*qz*qw, 1.0f - 2.0f*qx*qx - 2.0f*qz*qz, 2.0f*qy*qz - 2.0f*qx*qw,
            2.0f*qx*qz - 2.0f*qy*qw, 2.0f*qy*qz + 2.0f*qx*qw, 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;
    }

}



void templateSelectionCallback(const vigir_ocs_msgs::OCSObjectSelection::ConstPtr& newTemplate) {
    cout<<"I received a new template"<<endl;
    currentTemplateId = newTemplate->id;
}

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud) {

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pc_ (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*pointcloud, *pc_);

    //cout<<"Ich habe 1 Pointcloud empfangen: " << pc_->at(0) <<endl;

    currentTargetPointcloud = MatrixXf(3,pc_->size());

    for (int i = 0; i < pc_->size(); i++) {
        currentTargetPointcloud(0,i) = pc_->at(i).x;
        currentTargetPointcloud(1,i) = pc_->at(i).y;
        currentTargetPointcloud(2,i) = pc_->at(i).z;
    }
}

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

MatrixXf getTemplatePointcloud(string path, string filename) {
    std::ifstream file;
    string full_filename = path + filename; // TODO: / am Ende von path abfragen
    // TODO: Endung auf .pcd ueberpruefen

    cout<<"full filename: "<<full_filename<<endl;

    cout<<"cur dir: "<<get_working_path()<<endl;

    file.open(full_filename.c_str());
    if (!file.is_open()) {
        std::cerr<<"Error while reading the input file!"<<std::endl;
        return MatrixXf::Identity(3,3);
    }

    string tmp;
    do {
        file >> tmp;
    } while (tmp != "POINTS");

    int number_points;
    file >> number_points;

    file>>tmp;
    file>>tmp;

    MatrixXf pointcloud(3,number_points);
    for (int i = 1; i < number_points; i++) {
        file >> pointcloud(0,i);
        file >> pointcloud(1,i);
        file >> pointcloud(2,i);
    }

    return pointcloud;
}


// save test data to file to be able to work on icp without having to run the simulation every time
void saveDataToFile(VectorXf transformation, MatrixXf rotation, MatrixXf template_cloud, MatrixXf world_cloud) {
    ofstream transformationFile, rotationFile, templateFile, worldFile;

    transformationFile.open("transformation.txt");
    if (!transformationFile.is_open()) {
        cout<<"Fehler beim oeffnen von transformation.txt!"<<endl;
    }
    transformationFile << transformation(0)<<" ";
    transformationFile << transformation(1)<<" ";
    transformationFile << transformation(2);
    transformationFile.close();

    rotationFile.open("rotation.txt");
    if (!rotationFile.is_open()) {
        cout<<"Fehler beim oeffnen von rotation.txt!"<<endl;
    }
    rotationFile << rotation(0,0)<<" "<<rotation(0,1)<<" "<<rotation(0,2)<<" ";
    rotationFile << rotation(1,0)<<" "<<rotation(1,1)<<" "<<rotation(1,2)<<" ";
    rotationFile << rotation(2,0)<<" "<<rotation(2,1)<<" "<<rotation(2,2)<<" ";
    rotationFile.close();

    templateFile.open("template_cloud.txt");
    if (!templateFile.is_open()) {
        cout<<"Fehler beim oeffnen von template_cloud.txt!"<<endl;
    }
    templateFile << template_cloud.cols() << "\n";
    for (int i = 0; i < template_cloud.cols(); i++) {
        templateFile << template_cloud(0,i) << " ";
        templateFile << template_cloud(1,i) << " ";
        templateFile << template_cloud(2,i) << "\n";
    }
    templateFile.close();

    worldFile.open("world_cloud.txt");
    if (!worldFile.is_open()) {
        cout<<"Fehler beim oeffnen von world_cloud.txt!"<<endl;
    }
    worldFile << world_cloud.cols() << "\n";
    for (int i = 0; i < world_cloud.cols(); i++) {
        worldFile << world_cloud(0,i) << " ";
        worldFile << world_cloud(1,i) << " ";
        worldFile << world_cloud(2,i) << "\n";
    }
    worldFile.close();
}


// *****************************************************************************************************
// ************************************ ICP algorithm **************************************************
// *****************************************************************************************************


int get_inlier_number(float inlier_portion, int number_points) {
    return (int) floor(inlier_portion*((float) number_points));
}

float trimmed_scaling_icp(MatrixXf pc1, MatrixXf pc2, MatrixXf &R_result, VectorXf &t_result, float &s_result, MatrixXf R_init,
        VectorXf t_init, float s_init, float eps, int maxIt, float inlier_portion) {

    int itCt = 0;

    MatrixXf R_old = R_init;
    VectorXf t_old = t_init;
    float s_old = s_init;

    R_result = R_init;
    t_old = t_init;
    s_old = s_init;

    MatrixXf cp(pc1.rows(), pc1.cols());
    VectorXf distances(pc1.cols());
    MatrixXf pc1_proj(pc1.rows(), pc1.cols());

    int number_inliers = get_inlier_number(inlier_portion, pc1.cols());

    MatrixXf pc1_trimmed(3, number_inliers);
    MatrixXf cp_trimmed(3, number_inliers);

    apply_transformation(pc1, pc1_proj, R_init, t_init, s_init);

    while ((((R_result-R_old).norm() + (t_result-t_old).norm() + abs(s_result-s_old) > eps) || (itCt == 0)) && (itCt < maxIt)) {
        itCt++;
        cout<<"it: "<<itCt<<endl;

        R_old = R_result;
        t_old = t_result;
        s_old = s_result;

        find_correspondences(pc1_proj, pc2, cp, distances);

        trim_pc(pc1, cp, distances, pc1_trimmed, cp_trimmed, number_inliers);

        find_transformation(pc1_trimmed, cp_trimmed, R_result, t_result, s_result);

        apply_transformation(pc1, pc1_proj, R_result, t_result, s_result);
    }

    return calc_error(pc1_proj, pc2, inlier_portion);
}

void trim_pc(MatrixXf pc, MatrixXf cp, VectorXf distances, MatrixXf &pc_trimmed, MatrixXf &cp_trimmed, int number_inliers) {
    VectorXf distances_sorted = distances;

    sort(distances_sorted);

    float threshold = distances_sorted(number_inliers-1);

    int pos = 0;
    for (int i = 0; i < cp.cols(); i++) {
        if (distances(i) <= threshold && pos < number_inliers) {
            pc_trimmed(0,pos) = pc(0, i);
            pc_trimmed(1,pos) = pc(1, i);
            pc_trimmed(2,pos) = pc(2, i);

            cp_trimmed(0,pos) = cp(0, i);
            cp_trimmed(1,pos) = cp(1, i);
            cp_trimmed(2,pos) = cp(2, i);

            pos++;
        }
    }
}

void find_correspondences(MatrixXf pc1, MatrixXf pc2, MatrixXf &cp, VectorXf &distances) {
    MatrixXf distance_field(pc1.cols(), pc2.cols());
    MatrixXf distances_mat_i(3,pc2.cols());
    VectorXf distances_i(pc2.cols());

    int minIdx;

    for (int i = 0; i < pc1.cols(); i++) {
        distances_mat_i = pc2.array().colwise() - pc1.col(i).array();
        distances_i = distances_mat_i.row(0).cwiseProduct(distances_mat_i.row(0)) +
                      distances_mat_i.row(1).cwiseProduct(distances_mat_i.row(1)) +
                      distances_mat_i.row(2).cwiseProduct(distances_mat_i.row(2));
        distances_i = distances_i.array().sqrt();
        distances_i.minCoeff(&minIdx);

        distances(i) = distances_i(minIdx);

        cp(0,i) = pc2(0,minIdx);
        cp(1,i) = pc2(1,minIdx);
        cp(2,i) = pc2(2,minIdx);
    }
}

void find_transformation(MatrixXf pc1, MatrixXf pc2, MatrixXf &R, VectorXf &t, float &s) {
    VectorXf mean1 = pc1.array().rowwise().mean();
    VectorXf mean2 = pc2.array().rowwise().mean();

    MatrixXf pc1_norm = pc1.array().colwise() - mean1.array();
    MatrixXf pc2_norm = pc2.array().colwise() - mean2.array();

    MatrixXf W(3,3);
    W(0,0) = (pc1_norm.block(0,0,1,pc1.cols()) * pc2_norm.block(0,0,1,pc2.cols()).transpose())(0);
    W(0,1) = (pc1_norm.block(0,0,1,pc1.cols()) * pc2_norm.block(1,0,1,pc2.cols()).transpose())(0);
    W(0,2) = (pc1_norm.block(0,0,1,pc1.cols()) * pc2_norm.block(2,0,1,pc2.cols()).transpose())(0);

    W(1,0) = (pc1_norm.block(1,0,1,pc1.cols()) * pc2_norm.block(0,0,1,pc2.cols()).transpose())(0);
    W(1,1) = (pc1_norm.block(1,0,1,pc1.cols()) * pc2_norm.block(1,0,1,pc2.cols()).transpose())(0);
    W(1,2) = (pc1_norm.block(1,0,1,pc1.cols()) * pc2_norm.block(2,0,1,pc2.cols()).transpose())(0);

    W(2,0) = (pc1_norm.block(2,0,1,pc1.cols()) * pc2_norm.block(0,0,1,pc2.cols()).transpose())(0);
    W(2,1) = (pc1_norm.block(2,0,1,pc1.cols()) * pc2_norm.block(1,0,1,pc2.cols()).transpose())(0);
    W(2,2) = (pc1_norm.block(2,0,1,pc1.cols()) * pc2_norm.block(2,0,1,pc2.cols()).transpose())(0);



    JacobiSVD<MatrixXf> svd(W, ComputeThinU | ComputeThinV);

    MatrixXf U = -svd.matrixU();
    MatrixXf V = -svd.matrixV();

    R = U*V.transpose();
    R = R.inverse();

    if (R.determinant() < 0) {
        MatrixXf V = svd.matrixV();
        V(0,2) = -V(0,2);
        V(1,2) = -V(1,2);
        V(2,2) = -V(2,2);
        R = U*V.transpose();
        R = R.inverse();
        // TODO: eigenvalues abfragen
    }

    MatrixXf a = R*pc1_norm;
    MatrixXf b = pc2_norm;

    MatrixXf tmp1 = a.cwiseProduct(b);
    MatrixXf tmp2 = a.cwiseProduct(a);

    s = (((float) tmp1.rows())*((float) tmp1.cols())*tmp1.norm()) / (((float) tmp2.rows())*((float) tmp2.cols())*tmp2.norm());

    t = mean2 - s*R*mean1;
}

void apply_transformation(MatrixXf pc, MatrixXf &pc_proj, MatrixXf R, VectorXf t, float s) {
    pc_proj = s*R*pc;
    pc_proj = pc_proj.array().colwise() + t.array();
}

float calc_error(MatrixXf pc1, MatrixXf pc2, float inlier_portion) {
    int number_inliers = get_inlier_number(inlier_portion, pc1.cols());

    MatrixXf cp(3, pc1.cols());
    VectorXf distances(pc1.cols());
    MatrixXf pc1_trimmed(3, number_inliers);
    MatrixXf cp_trimmed(3, number_inliers);

    find_correspondences(pc1, pc2, cp, distances);
    trim_pc(pc1, cp, distances, pc1_trimmed, cp_trimmed, number_inliers);
    return calc_error(pc1_trimmed, cp_trimmed);
}

float calc_error(MatrixXf pc1, MatrixXf pc2) {
    MatrixXf diff = pc1 - pc2;
    diff = diff.row(0).cwiseProduct(diff.row(0)) + diff.row(1).cwiseProduct(diff.row(1)) + diff.row(2).cwiseProduct(diff.row(2));
    diff = diff.array().sqrt();

    return diff.sum();
}

void sort(VectorXf &v) {
  std::sort(v.data(), v.data()+v.size());
}

VectorXf matrixToVector(MatrixXf m) {
    m.transposeInPlace();
    VectorXf v(Map<VectorXf>(m.data(), m.cols()*m.rows()));
    return v;
}

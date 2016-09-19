#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <object_template_alignment_plugin/PointcloudAlignmentAction.h>
#include <geometry_msgs/PoseStamped.h>

#include <string>
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <float.h>

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

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>


using namespace Eigen;
using namespace std;

typedef struct Cube {
    VectorXf r0;
    float half_edge_length;
    float lower_bound;
    float upper_bound;
    int depth;
} Cube;

typedef struct QueueElement {
    Cube *cube;
    struct QueueElement *next;
} QueueElement;

typedef struct PriorityQueue {
    QueueElement *head;
} PriorityQueue;


MatrixXf getTemplatePointcloud(string path, string filename);
void saveDataToFile(VectorXf transformation, MatrixXf rotation, MatrixXf template_cloud, MatrixXf world_cloud);

float find_pointcloud_alignment(MatrixXf pc1, MatrixXf pc2, float eps, VectorXf t_init, float s_init, MatrixXf &R_icp, VectorXf &t_icp, float &s_icp,
    int &itCt, int maxIcpIt, float icpEps, int maxDepth, float inlier_portion, float maxTime);
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

void getEulerAxisRotationMatrix(VectorXf r, MatrixXf &R);
Cube **splitCube(Cube *cube);
int depthNumber(PriorityQueue *Q, int depth);
int *getPermutation(int start, int end);
Cube **createPriorityQueue(int maxDepth, int &queueLength);
void fillPriorityQueue(PriorityQueue *Q, Cube *cube, int curDepth, int maxDepth);
PriorityQueue *createQueue();
void insert(PriorityQueue *queue, Cube *cube);
void deleteQueue(PriorityQueue *queue);
int length(PriorityQueue *queue);
Cube *extractFirstElement(PriorityQueue *queue);
float getPassedTime(struct timeval start);
MatrixXf getAARot(VectorXf r);
Cube* createCube(VectorXf r0, float half_edge_length, int depth);
bool betterThan(Cube *cube1, Cube *cube2);

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > random_filter(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pc, int number_points);
MatrixXf random_filter(MatrixXf pc, int number_points);



static int currentTemplateId;
static VectorXf currentPosition;
static MatrixXf currentRotation;
static MatrixXf currentTargetPointcloud;
static pcl::KdTreeFLANN<pcl::PointXYZ> targetKdTree;
static bool pointcloudReceived = false, templateReceived = false, templateListReceived = false;

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
        if (pointcloudReceived == false || templateReceived == false || templateListReceived == false) {
            if (pointcloudReceived == false) {
                ROS_ERROR("No pointcloud received - Please send a pointcloud request first.");
            }
            if (templateReceived == false) {
                ROS_ERROR("No template received - Please choose template and double click on it.");
            }
            if (templateListReceived == false) {
                ROS_ERROR("No template position available - Please move the template to send position signal.");
            }

            return;
        }

        bool success = true;

        std::string filename = "DRC_drill.pcd";
        //std::string path = "~/thor/src/vigir/vigir_templates/vigir_template_library/object_library/tools/";
        std::string path = "vigir/vigir_templates/vigir_template_library/object_library/tools/";

        MatrixXf template_pointcloud = getTemplatePointcloud(path, filename);

        template_pointcloud = random_filter(template_pointcloud, 200);


        /*float discardPointThreshold = 1.5;
        float maxRadius = FLT_MIN;
        VectorXf center(3), curPoint(3);
        center << 0, 0, 0;

        for (int i = 0; i < pc_->size(); i++) {
            curPoint << pc_->at(i).x, pc_->at(i).y, pc_->at(i).z;
            center = (center*((float) i) + curPoint)/((float) i+1);
        }

        for (int i = 0; i < pc_->size; i++) {
            curPoint << pc_->at(i).x, pc_->at(i).y, pc_->at(i).z;
            float curRadius = (curPoint-center).norm();

            if (curRadius < maxRadius) {
                maxRadius = curRadius;
            }
        }

        for (int i = pc_->size()-1; i >= 0; i--) {
            curPoint << pc_->at(i).x, pc_->at(i).y, pc_->at(i).z;
            float curRadius = (curPoint-center).norm();

            if (curRadius > th)
        }*/




        MatrixXf R_icp(3,3);
        VectorXf t_icp(3);
        float s_icp;


        R_icp;
        R_icp = currentRotation;
        t_icp = currentPosition;
        s_icp = 1.;

        int itCt = 0;

        //float error = find_pointcloud_alignment(template_pointcloud, currentTargetPointcloud, 1, currentPosition, 1., R_icp, t_icp, s_icp,
        //                                             itCt, 300, 1e-4, 1, 0.4, 10);
        //cout<<"icp error: "<<error<<endl;

        //saveDataToFile(currentPosition, currentRotation, template_pointcloud, currentTargetPointcloud);

        //t_icp = currentPosition;
        //R_icp = MatrixXf::Identity(3,3);

        float error = trimmed_scaling_icp(template_pointcloud, currentTargetPointcloud, R_icp, t_icp, s_icp, currentRotation, currentPosition, 1.0, 1e-7, 300, 0.4);

        cout<<"error: "<<error<<endl;

        //saveDataToFile(currentPosition, currentRotation, template_pointcloud, currentTargetPointcloud);

        //R_icp = currentRotation;
        //t_icp = currentPosition;
        //s_icp = 1.;



        if (success)
        {
        geometry_msgs::Quaternion orientation;
        geometry_msgs::Point position;

        position.x = t_icp(0);
        position.y = t_icp(1);
        position.z = t_icp(2);

        orientation.w = sqrt(1. + R_icp(0,0) + R_icp(1,1) + R_icp(2,2)) / 2.; // TODO: Division durch Null abfangen
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

    templateListReceived = true;
}



void templateSelectionCallback(const vigir_ocs_msgs::OCSObjectSelection::ConstPtr& newTemplate) {
    cout<<"I received a new template"<<endl;
    currentTemplateId = newTemplate->id;

    templateReceived = true;
}

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud) {



    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pc_ (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*pointcloud, *pc_);

    //pc_ = random_filter(pc_, 2000);

    //cout<<"Ich habe 1 Pointcloud empfangen: " << pc_->at(0) <<endl;

    currentTargetPointcloud = MatrixXf(3,pc_->size());

    for (int i = 0; i < pc_->size(); i++) {
        currentTargetPointcloud(0,i) = pc_->at(i).x;
        currentTargetPointcloud(1,i) = pc_->at(i).y;
        currentTargetPointcloud(2,i) = pc_->at(i).z;
    }


    targetKdTree.setInputCloud (pc_);

    pointcloudReceived = true;
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

float find_pointcloud_alignment(MatrixXf pc1, MatrixXf pc2, float eps, VectorXf t_init, float s_init, MatrixXf &R_icp, VectorXf &t_icp, float &s_icp,
    int &itCt, int maxIcpIt, float icpEps, int maxDepth, float inlier_portion, float maxTime) {

    struct timeval start;
    gettimeofday(&start, NULL);

    int queueLength;
    Cube **Q = createPriorityQueue(maxDepth, queueLength);

    volatile bool success = false;

    itCt = 0;
    float minErr = FLT_MAX;








    VectorXf t_star(3);
    MatrixXf R_star(3,3);
    t_star << 0.453278, -0.00371148, 0.90559;
    R_star << 0.0155734, -0.999806, 0.0120183, 0.999878, 0.0155607, -0.00114424, 0.00095701, 0.0120346, 0.999927;





    //#pragma omp parallel for shared(success, R_icp, t_icp, s_icp)
    for (int i = 0; i < queueLength; i++) {
        // send feedback
        //PointcloudAlignmentAction::feedback_.percentage = ((float) i) / ((float) queueLength);
        cout<<"percentage: "<< ((float) i) / ((float) queueLength)*100.<<"%, iteration "<<i<<"/"<<queueLength << endl;

        if (i != 0 && getPassedTime(start) > maxTime) {
            break;
        }

        MatrixXf R_i(3,3);
        VectorXf t_i(3);
        float s_i;

        //int number_inliers = get_inlier_number(inlier_portion, pc1.cols());

        float err = trimmed_scaling_icp(pc1, pc2, R_i, t_i, s_i, getAARot(Q[itCt++]->r0), t_init, s_init, icpEps, maxIcpIt, inlier_portion);

        float err_star = (R_star-R_i).norm() + (t_star - t_i).norm() + abs(1.-s_i);

        //if (err < minErr && s_i > 0.1) {
        if (err_star < minErr) {
            minErr = err;

            R_icp = R_i;
            t_icp = t_i;
            s_icp = s_i;
        }

        /*#pragma omp critical
        {
            if (err < eps && s_i > 0.8) {
                error = err;

                R_icp = R_i;
                t_icp = t_i;
                s_icp = s_i;

                cout<<"success!!!"<<endl;
                cout<<R_icp<<endl<<t_icp<<endl<<s_icp<<endl;

                success = true;
            }
        }*/
    }

    //R_icp = R_star;
    //t_icp = t_star;
    //s_icp = 1.;

    return minErr;
}

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
        //cout<<"it: "<<itCt<<endl;

        R_old = R_result;
        t_old = t_result;
        s_old = s_result;

        find_correspondences(pc1_proj, pc2, cp, distances);

        trim_pc(pc1, cp, distances, pc1_trimmed, cp_trimmed, number_inliers);

        find_transformation(pc1_trimmed, cp_trimmed, R_result, t_result, s_result);

        apply_transformation(pc1, pc1_proj, R_result, t_result, s_result);
    }

    cout<<"iterations needed: "<<itCt<<endl;
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
    pcl::PointXYZ searchPoint;

    for (int i = 0; i < pc1.cols(); i++) {
        searchPoint.x = pc1(0,i);
        searchPoint.y = pc1(1,i);
        searchPoint.z = pc1(2,i);

        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        if (targetKdTree.nearestKSearch (searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            for (size_t k = 0; k < pointIdxNKNSearch.size (); ++k) {
                cp(0,i) = pc2(0,pointIdxNKNSearch[k]);
                cp(1,i) = pc2(1,pointIdxNKNSearch[k]);
                cp(2,i) = pc2(2,pointIdxNKNSearch[k]);

                distances(i) = sqrt(pointNKNSquaredDistance[k]);
            }
        }
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

        V(0,2) = -V(0,2);
        V(1,2) = -V(1,2);
        V(2,2) = -V(2,2);
        R = V*svd.matrixU().transpose();
    }

    t = mean2 - R*mean1;
    s = 1.;

    /*JacobiSVD<MatrixXf> svd(W, ComputeThinU | ComputeThinV);

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

    t = mean2 - s*R*mean1;*/
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

void getEulerAxisRotationMatrix(VectorXf r, MatrixXf &R) {
    R = MatrixXf::Identity(3,3);
    if (r.norm() == 0) {
        return;
    }

    MatrixXf r_x(3,3);
    r_x << 0, -r(2), r(1),
        r(2), 0, -r(0),
        -r(1), r(0), 0;

    R += (r_x*sin(r.norm()))/(r.norm());
    R += (r_x*r_x*(1-cos(r.norm())))/(r.norm()*r.norm());
}

Cube **splitCube(Cube *cube) {
    Cube **subcubes = new Cube*[8];
    VectorXf offset(3);
    float hel = cube->half_edge_length/2.;
    float signs[2] = {-1.,+1.};

    int position = 0;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                offset << hel*signs[i],hel*signs[j],hel*signs[k];
                subcubes[position++] = createCube(cube->r0 + offset, hel, cube->depth + 1);
            }
        }
    }

    return subcubes;
}

int depthNumber(PriorityQueue *Q, int depth) {
    int ct = 0;
    QueueElement *tmp = Q->head;
    while (tmp != NULL) {
        if (tmp->cube->depth == depth) {
            ct++;
        }
        tmp = tmp->next;
    }
    return ct;
}

int *getPermutation(int start, int end) {
    int length = end-start+1;

    vector<int> indices;
    for (int i = 0; i < length; i++) {
        indices.push_back(start+i);
    }
    random_shuffle(indices.begin(), indices.end());

    int *permutation = new int[length];
    for (int i = 0; i < length; i++) {
        permutation[i] = indices[i];
    }

    return permutation;
}

Cube **createPriorityQueue(int maxDepth, int &queueLength) {
    PriorityQueue *Q = createQueue();
    VectorXf r0_init(3);
    r0_init<<0,0,0;
    Cube *C_init = createCube(r0_init, M_PI, 0);
    insert(Q, C_init);
    fillPriorityQueue(Q, C_init, 0, maxDepth);
    int nCubes = length(Q);
    Cube **priorityQueue = new Cube *[nCubes];
    int depth = 0;
    int startPos = 0;
    while (depth <= maxDepth) {
        int ct = depthNumber(Q, depth++);
        int *offset = getPermutation(0, ct-1);

        for (int i = 0; i < ct; i++) {
            Cube *tmp = extractFirstElement(Q);
            priorityQueue[startPos+offset[i]] = tmp;
        }
        startPos += ct;
    }

    queueLength = nCubes;

    return priorityQueue;
}

void fillPriorityQueue(PriorityQueue *Q, Cube *cube, int curDepth, int maxDepth) {
    float vecLength = sqrt(cube->r0[0]*cube->r0[0] + cube->r0[1]*cube->r0[1] + cube->r0[2]*cube->r0[2]);

    if (vecLength < M_PI) {
        insert(Q, cube);
    }

    if (curDepth < maxDepth) {
        Cube **subcubes = splitCube(cube);

        for (int i = 0; i < 8; i++) {
            fillPriorityQueue(Q, subcubes[i], curDepth + 1, maxDepth);
        }
    }
}

PriorityQueue *createQueue() {
    PriorityQueue *queue = new PriorityQueue;
    queue->head = NULL;
    return queue;
}

void insert(PriorityQueue *queue, Cube *cube) {
    if (queue == NULL) {
        perror("queue == NULL");
        return;
    }
    QueueElement *newElement = new QueueElement;
    newElement->cube = cube;

    if (queue->head == NULL) {
        queue->head = newElement;
        queue->head->next = NULL;
        return;

    } else {
        if (betterThan(cube, queue->head->cube) == true) {
            newElement->next = queue->head;
            queue->head = newElement;
            return;
        }
        QueueElement *tmp = queue->head;
        while (tmp->next != 0) {
            if (betterThan(cube, tmp->next->cube) == true) {
                newElement->next = tmp->next;
                tmp->next = newElement;
                return;;
            }

            tmp = tmp->next;
        }
        tmp->next = newElement;
        newElement->next = NULL;
    }
}

void deleteQueue(PriorityQueue *queue) {
    if (queue == NULL)
        return;
    if (queue->head == NULL)
        free(queue);
    QueueElement *temp = queue->head, *next = NULL;
    while (temp != NULL) {
        next = temp->next;
        delete(temp);
        temp = next;
    }
    delete(queue);
}

int length(PriorityQueue *queue) {
    if (queue == NULL || queue->head == NULL)
        return 0;

    int counter = 0;
    QueueElement *temp = queue->head;
    while (temp != NULL) {
        counter++;
        temp = temp->next;
    }
    return counter;
}

Cube *extractFirstElement(PriorityQueue *queue) {
    if (queue == NULL || queue->head == NULL)
        return NULL;

    QueueElement *element = queue->head;
    Cube *cube = element->cube;
    queue->head = queue->head->next;
    delete(element);

    return cube;
}

float getPassedTime(struct timeval start) {
    struct timeval end;
    gettimeofday(&end, NULL);

    return (float) (((1.0/1000)*((end.tv_sec * 1000000 + end.tv_usec) - (start.tv_sec * 1000000 + start.tv_usec)))/1000.);
}

MatrixXf getAARot(VectorXf r) {
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

Cube* createCube(VectorXf r0, float half_edge_length, int depth) {
    Cube *C = new Cube;
    C->r0 = r0;
    C->half_edge_length = half_edge_length;
    C->depth = depth;

    return C;
}

bool betterThan(Cube *cube1, Cube *cube2) {
    if (cube1->depth < cube2->depth) {
        return true;
    } else if (cube1->depth == cube2->depth && cube1->lower_bound < cube2->lower_bound) {
        return true;
    } else {
        return false;
    }
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > random_filter(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pc, int number_points) {
    if (number_points > pc->size()) {
        return pc;
    }

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pc_filtered;
    cout<<"hier"<<endl;
    pc_filtered->width = 2000;
    //pc_filtered->width = number_points;
    cout<<"hier"<<endl;
    pc_filtered->height   = 1;
    cout<<"hier"<<endl;
    pc_filtered->is_dense = pc->is_dense;
    cout<<"hier"<<endl;
    pc_filtered->points.resize (pc_filtered->width * pc_filtered->height);
    cout<<"hier"<<endl;

    vector<int> indices;
    for (int i = 0; i < pc->size(); i++) {
        indices.push_back(i);
    }
    random_shuffle(indices.begin(), indices.end());

    for (int i = 0; i < number_points; i++) {
        cout<<"i: "<<i<<endl;
        pc_filtered->points[i].x = pc->at(indices[i]).x;
        pc_filtered->points[i].y = pc->at(indices[i]).y;
        pc_filtered->points[i].z = pc->at(indices[i]).z;

    }

}

MatrixXf random_filter(MatrixXf pc, int number_points) {
    if (number_points > pc.cols()) {
        return pc;
    }

    vector<int> indices;
    for (int i = 0; i < pc.cols(); i++) {
        indices.push_back(i);
    }
    random_shuffle(indices.begin(), indices.end());

    MatrixXf filtered_pc(pc.rows(), number_points);
    for (int i = 0; i < number_points; i++) {
        filtered_pc(0,i) = pc(0, indices[i]);
        filtered_pc(1,i) = pc(1, indices[i]);
        filtered_pc(2,i) = pc(2, indices[i]);
    }

    return filtered_pc;
}

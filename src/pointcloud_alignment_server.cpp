#include <ros/ros.h>
#include <ros/package.h>
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

#include <omp.h>


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

static int currentTemplateId; // TODO: delete


MatrixXf getTemplatePointcloud(string path, string filename);
void saveDataToFile(VectorXf transformation, MatrixXf rotation, MatrixXf template_cloud, MatrixXf world_cloud);

float find_pointcloud_alignment(int command, MatrixXf template_pointcloud, MatrixXf world_pointcloud, MatrixXf &R_icp, VectorXf &t_icp, float &s_icp);
float local_pointcloud_alignment(int number_subclouds, MatrixXf *source_subclouds, MatrixXf target_pointcloud, MatrixXf &R, VectorXf &t, float &s);
float global_pointcloud_alignment(int number_subclouds, MatrixXf *source_subclouds, MatrixXf target_pointcloud, MatrixXf &R, VectorXf &t, float &s);

int get_inlier_number(float inlier_portion, int number_points);
void trim_pointcloud(MatrixXf pointcloud, MatrixXf correspondences, VectorXf distances, MatrixXf &pointcloud_trimmed, MatrixXf &correspondences_trimmed);
void find_correspondences(MatrixXf source_pointcloud, MatrixXf target_pointcloud, MatrixXf &correspondences, VectorXf &distances);
void find_transformation(MatrixXf source_pointcloud, MatrixXf target_pointcloud, MatrixXf &R, VectorXf &t, float &s);
void apply_transformation(MatrixXf pointcloud, MatrixXf &pointcloud_proj, MatrixXf R, VectorXf t, float s);
//float calc_error(MatrixXf source_pointcloud, MatrixXf target_pointcloud, float inlier_portion);
float calc_error(MatrixXf source_pointcloud, MatrixXf target_pointcloud);
void sort(VectorXf &v);
VectorXf matrixToVector(MatrixXf m);

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

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > random_filter(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pointcloud, int number_points);
MatrixXf random_filter(MatrixXf pointcloud, int number_points);

float noise(float range);
void addNoiseToParams(MatrixXf &R, VectorXf &t, float &s, float range);




static pcl::KdTreeFLANN<pcl::PointXYZ>targetKdTree;

static const float INLIER_PORTION = 0.5;


class PointcloudAlignmentAction
{
private:



protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<object_template_alignment_plugin::PointcloudAlignmentAction> as_;
    std::string action_name_;
    object_template_alignment_plugin::PointcloudAlignmentFeedback feedback_;
    object_template_alignment_plugin::PointcloudAlignmentResult result_;
    ros::Subscriber sub1_, sub2_, sub3_;

public:
    PointcloudAlignmentAction(std::string name) :
    as_(nh_, name, boost::bind(&PointcloudAlignmentAction::executeCB, this, _1), false),
    action_name_(name) {

        sub1_ = nh_.subscribe("/flor/worldmodel/ocs/cloud_result", 1000, &PointcloudAlignmentAction::pointcloudCallback, this);

        sub2_ = nh_.subscribe("/flor/ocs/object_selection", 1000, &PointcloudAlignmentAction::templateSelectionCallback, this);

        sub3_ = nh_.subscribe("/template/list", 1000, &PointcloudAlignmentAction::templateListCallback, this);

        as_.start();
    }

    ~PointcloudAlignmentAction(void) {}

    void templateListCallback(const vigir_object_template_msgs::TemplateServerList::ConstPtr& templateList) {

    }

    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud) {

    }

    void templateSelectionCallback(const vigir_ocs_msgs::OCSObjectSelection::ConstPtr& newTemplate) {
        cout<<"I received a new template"<<endl;
        currentTemplateId = newTemplate->id;
    }



    void executeCB(const object_template_alignment_plugin::PointcloudAlignmentGoalConstPtr &goal) {
        int TARGET_RADIUS_FACTOR = 1.3;

        // TODO: Check if given arguments are valid

        // convert messages to pointclouds
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pointcloud_source (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(goal->source_pointcloud , *pointcloud_source);

        MatrixXf source_pointcloud = MatrixXf(3,pointcloud_source->size());

        float max_radius = FLT_MIN;

        for (int i = 0; i < pointcloud_source->size(); i++) {
            source_pointcloud(0,i) = pointcloud_source->at(i).x;
            source_pointcloud(1,i) = pointcloud_source->at(i).y;
            source_pointcloud(2,i) = pointcloud_source->at(i).z;

            float radius = sqrt(pointcloud_source->at(i).x*pointcloud_source->at(i).x +
                                pointcloud_source->at(i).y*pointcloud_source->at(i).y +
                                pointcloud_source->at(i).z*pointcloud_source->at(i).z);
            if (radius > max_radius) {
                max_radius = radius;
            }
        }


        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pointcloud_target (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(goal->target_pointcloud, *pointcloud_target);


        int target_size = 0;
        for (int i = 0; i < pointcloud_target->size(); i++) {
            float dist = sqrt(pow(pointcloud_target->at(i).x - goal->initial_pose.pose.position.x,2) +
                              pow(pointcloud_target->at(i).y - goal->initial_pose.pose.position.y,2) +
                              pow(pointcloud_target->at(i).z - goal->initial_pose.pose.position.z,2));

            if (dist < TARGET_RADIUS_FACTOR*max_radius) {
                target_size++;
            }

        }

        MatrixXf target_pointcloud = MatrixXf(3,target_size);

        pcl::PointCloud<pcl::PointXYZ>::Ptr target_pcl_pointcloud (new pcl::PointCloud<pcl::PointXYZ>);;

        // Fill in the cloud data
        target_pcl_pointcloud->width    = target_size;
        target_pcl_pointcloud->height   = 1;
        target_pcl_pointcloud->is_dense = false;
        target_pcl_pointcloud->points.resize(target_pcl_pointcloud->width * target_pcl_pointcloud->height);

        int pos = 0;
        for (int i = 0; i < target_size; i++) {
            float dist = sqrt(pow(pointcloud_target->at(i).x - goal->initial_pose.pose.position.x,2) +
                              pow(pointcloud_target->at(i).y - goal->initial_pose.pose.position.y,2) +
                              pow(pointcloud_target->at(i).z - goal->initial_pose.pose.position.z,2));

            if (dist < TARGET_RADIUS_FACTOR*max_radius) {
                target_pointcloud(0,pos) = pointcloud_target->at(i).x;
                target_pointcloud(1,pos) = pointcloud_target->at(i).y;
                target_pointcloud(2,pos) = pointcloud_target->at(i).z;

                target_pcl_pointcloud->points[pos].x = pointcloud_target->at(i).x;
                target_pcl_pointcloud->points[pos].y = pointcloud_target->at(i).y;
                target_pcl_pointcloud->points[pos].z = pointcloud_target->at(i).z;

                pos++;
            }
        }

        targetKdTree.setInputCloud (target_pcl_pointcloud); // TODO: als Argument uebergeben



        VectorXd s_tmp(3), t_tmp(3);
        s_tmp << 0,0,0;
        t_tmp << 0,0,0;
        for (int i = 0; i < source_pointcloud.cols(); i++) {
            if (i % 2 == 0) {
                s_tmp(0) += source_pointcloud(0,i);
                s_tmp(1) += source_pointcloud(1,i);
                s_tmp(2) += source_pointcloud(2,i);
            } else {
                s_tmp(0) -= source_pointcloud(0,i);
                s_tmp(1) -= source_pointcloud(1,i);
                s_tmp(2) -= source_pointcloud(2,i);
            }
        }

        for (int i = 0; i < target_pointcloud.cols(); i++) {
            if (i % 2 == 0) {
                t_tmp(0) += target_pointcloud(0,i);
                t_tmp(1) += target_pointcloud(1,i);
                t_tmp(2) += target_pointcloud(2,i);
            } else {
                t_tmp(0) -= target_pointcloud(0,i);
                t_tmp(1) -= target_pointcloud(1,i);
                t_tmp(2) -= target_pointcloud(2,i);
            }
        }

        cout<<"s: "<<s_tmp<<endl<<"t_tmp: "<<t_tmp<<endl;

        // convert initial_pose structure to transformation parameters
        MatrixXf R_icp = MatrixXf(3,3);
        float qx = goal->initial_pose.pose.orientation.x;
        float qy = goal->initial_pose.pose.orientation.y;
        float qz = goal->initial_pose.pose.orientation.z;
        float qw = goal->initial_pose.pose.orientation.w;
        R_icp <<
                 1.0f - 2.0f*qy*qy - 2.0f*qz*qz, 2.0f*qx*qy - 2.0f*qz*qw, 2.0f*qx*qz + 2.0f*qy*qw,
                 2.0f*qx*qy + 2.0f*qz*qw, 1.0f - 2.0f*qx*qx - 2.0f*qz*qz, 2.0f*qy*qz - 2.0f*qx*qw,
                 2.0f*qx*qz - 2.0f*qy*qw, 2.0f*qy*qz + 2.0f*qx*qw, 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;

        VectorXf t_icp(3);
        t_icp << goal->initial_pose.pose.position.x, goal->initial_pose.pose.position.y, goal->initial_pose.pose.position.z;
        float s_icp = 1.;


        // execute the pointcloud alignment algorithm
        float error= find_pointcloud_alignment(goal->command, source_pointcloud, target_pointcloud, R_icp, t_icp, s_icp);
        cout<<"error: "<<error<<endl;


        // send goal
        geometry_msgs::Quaternion orientation;
        geometry_msgs::Point position;

        position.x = t_icp(0);
        position.y = t_icp(1);
        position.z = t_icp(2);

        if (R_icp(0,0) + R_icp(1,1) + R_icp(2,2) == 0) {
            ROS_ERROR("Received invalid Rotation matrix!");
            as_.setAborted();
        }

        orientation.w = sqrt(1. + R_icp(0,0) + R_icp(1,1) + R_icp(2,2)) / 2.;
        orientation.x = (R_icp(2,1) - R_icp(1,2)) / (4.*orientation.w);
        orientation.y = (R_icp(0,2) - R_icp(2,0)) / (4.*orientation.w);
        orientation.z = (R_icp(1,0) - R_icp(0,1)) / (4.*orientation.w);

        geometry_msgs::PoseStamped result;

        result.pose.orientation = orientation;
        result.pose.position = position;

        result_.transformation_pose = result;

        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_);
    }


    float find_pointcloud_alignment(int command, MatrixXf source_pointcloud, MatrixXf target_pointcloud, MatrixXf &R_icp, VectorXf &t_icp, float &s_icp) {

        // create different sample versions of the source cloud
        int number_subclouds = 5;
        int size_source = 250;

        int size_target = 500;

        MatrixXf *source_subclouds = new MatrixXf[number_subclouds];

        for (int i = 0; i < number_subclouds; i++) {
            source_subclouds[i] = random_filter(source_pointcloud, size_source);
        }

        target_pointcloud = random_filter(target_pointcloud, size_target);


        // execute algorithm accoring to command
        if (command == 0) { // execute local icp
            cout<<"executing local icp"<<endl;
            return  local_pointcloud_alignment(number_subclouds, source_subclouds, target_pointcloud, R_icp, t_icp, s_icp);
        } else  if (command == 1) { // execute global pointcloud alignment
            cout<<"executing global icp"<<endl;
            return global_pointcloud_alignment(number_subclouds, source_subclouds, target_pointcloud, R_icp, t_icp, s_icp);

        } else { // invalid command
            ROS_ERROR("Received invalid command: %d", command);
            as_.setAborted();
        }
    }

    float global_pointcloud_alignment(int number_subclouds, MatrixXf *source_subclouds, MatrixXf target_pointcloud, MatrixXf &R, VectorXf &t, float &s) {
        float maxTime = 1.5;
        int maxDepth = 2;


        struct timeval start;
        gettimeofday(&start, NULL);

        int queueLength;
        Cube **Q = createPriorityQueue(maxDepth, queueLength);

        MatrixXf R_init = R;
        VectorXf t_init = t;
        float s_init = s;

        int itCt = 0;
        float minErr = FLT_MAX;

        #pragma omp parallel for shared(minErr, R, t, s)
        for (int i = 0; i < queueLength; i++) {
            // TODO: send feedback

            if (i != 0 && getPassedTime(start) > maxTime) {
                continue;
            }

            MatrixXf R_i = getAARot(Q[i]->r0) * R_init;
            VectorXf t_i = t_init;
            float s_i = s_init;

            float err = local_pointcloud_alignment(number_subclouds, source_subclouds, target_pointcloud, R_i, t_i, s_i);

            if (err < minErr && s_i > 0.8) {
                minErr = err;

                R = R_i;
                t = t_i;
                s = s_i;
            }

            itCt++;
        }

        R_init = R;
        t_init = t;
        s_init = s;

        MatrixXf R_sym[3];
        R_sym[0] = getRotationMatrix(M_PI,0,0);
        R_sym[1] = getRotationMatrix(0,M_PI,0);
        R_sym[2] = getRotationMatrix(0,0,M_PI);

        #pragma omp parallel for shared(minErr, R, t, s)
        for (int i = 0; i < 3; i++) {
            MatrixXf R_i = R_sym[0];
            VectorXf t_i = t_init;
            float s_i = s_init;

            float err = local_pointcloud_alignment(number_subclouds, source_subclouds, target_pointcloud, R_i, t_i, s_i);

            if (err < minErr && s_i > 0.8) {
                minErr = err;

                R = R_i;
                t = t_i;
                s = s_i;
            }

            itCt++;
        }

        cout<<"Executed "<<itCt<<" + icp iterations."<<endl;
        return minErr;
    }


    float local_pointcloud_alignment(int number_subclouds, MatrixXf *source_subclouds, MatrixXf target_pointcloud, MatrixXf &R, VectorXf &t, float &s) {

        float eps = 1e-7;
        int maxIt = 300;
        float err_eps = 0.1;

        int source_size = source_subclouds[0].cols();
        int number_inliers = get_inlier_number(INLIER_PORTION, source_size);

        float err_old;
        int itCt = 0;

        MatrixXf correspondences(3, source_size);
        VectorXf distances(source_size);
        MatrixXf source_proj(3, source_size);
        MatrixXf source_trimmed(3, number_inliers);
        MatrixXf correspondences_trimmed(3, number_inliers);
        MatrixXf R_old(3,3);
        VectorXf t_old(3);
        float s_old;

        int source_pos = 0;

        MatrixXf source_cloud = source_subclouds[source_pos];


        while(itCt < maxIt) {

            source_cloud = source_subclouds[source_pos % number_subclouds];
            itCt++;

            R_old = R;
            t_old = t;
            s_old = s;

            apply_transformation(source_cloud, source_proj, R, t, s);

            find_correspondences(source_proj, target_pointcloud, correspondences, distances);

            trim_pointcloud(source_cloud, correspondences, distances, source_trimmed, correspondences_trimmed);

            find_transformation(source_trimmed, correspondences_trimmed, R, t, s);

            if ((R-R_old).norm() + (t-t_old).norm() + abs(s-s_old) < eps) {
                if (source_pos == 0) {
                    err_old = calc_error(source_cloud, target_pointcloud, R, t, s);
                } else if (source_pos % 5 == 0) {
                    float err = calc_error(source_cloud, target_pointcloud, R, t, s);
                    if (abs(err-err_old) < err_eps) {
                        break;
                    } else {
                        err_old = err;
                    }
                }
                source_pos++;

            }
        }

        float err = calc_error(source_subclouds[0], target_pointcloud, R, t, s);
        cout<<"icp err: "<<err<<endl;
        return err;
    }

    void printDistances(VectorXf distances) {
        ofstream file;

        file.open("/home/sebastian/Desktop/distances.txt");
        if (!file.is_open()) {
            cout<<"Fehler beim oeffnen von distances.txt!"<<endl;
        }

        for (int i = 0; i < distances.rows(); i++) {
            file << distances(i) << endl;
        }

        file.close();
    }

    int pointsLowerThanThreshold(MatrixXf source, MatrixXf target, float threshold) {
        MatrixXf correspondences(source.rows(), source.cols());
        VectorXf distances(source.cols());
        int number = 0;

        find_correspondences(source, target, correspondences, distances);

        for (int i = 0; i < source.cols(); i++) {
            if (distances(i) < threshold) {
                number++;
            }
        }

        printDistances(distances);

        return number;
    }

    void trim_pointcloud(MatrixXf pointcloud, MatrixXf correspondences, VectorXf distances, MatrixXf &pointcloud_trimmed, MatrixXf &correspondences_trimmed) {
        int number_inliers = get_inlier_number(INLIER_PORTION, pointcloud.cols());

        VectorXf distances_sorted = distances;

        sort(distances_sorted);

        float threshold = distances_sorted(number_inliers-1);

        int pos = 0;
        for (int i = 0; i < correspondences.cols(); i++) {
            if (distances(i) <= threshold && pos < number_inliers) {
                pointcloud_trimmed(0,pos) = pointcloud(0, i);
                pointcloud_trimmed(1,pos) = pointcloud(1, i);
                pointcloud_trimmed(2,pos) = pointcloud(2, i);

                correspondences_trimmed(0,pos) = correspondences(0, i);
                correspondences_trimmed(1,pos) = correspondences(1, i);
                correspondences_trimmed(2,pos) = correspondences(2, i);

                pos++;
            }
        }
    }

    /*void find_correspondences(MatrixXf source_pointcloud, MatrixXf target_pointcloud, MatrixXf &correspondences, VectorXf &distances) {
        pcl::PointXYZ searchPoint;

        for (int i = 0; i < source_pointcloud.cols(); i++) {
            searchPoint.x = source_pointcloud(0,i);
            searchPoint.y = source_pointcloud(1,i);
            searchPoint.z = source_pointcloud(2,i);

            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);
            if (targetKdTree.nearestKSearch (searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                correspondences(0,i) = target_pointcloud(0,pointIdxNKNSearch[0]);
                correspondences(1,i) = target_pointcloud(1,pointIdxNKNSearch[0]);
                correspondences(2,i) = target_pointcloud(2,pointIdxNKNSearch[0]);

                distances(i) = sqrt(pointNKNSquaredDistance[0]);
            }
        }
    }*/

    void find_correspondences(MatrixXf pc1, MatrixXf pc2, MatrixXf &cp, VectorXf &distances) {
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

    void find_transformation(MatrixXf pointcloud, MatrixXf correspondences, MatrixXf &R, VectorXf &t, float &s) {
        VectorXf mean1 = pointcloud.array().rowwise().mean();
        VectorXf mean2 = correspondences.array().rowwise().mean();

        MatrixXf pointcloud_norm = pointcloud.array().colwise() - mean1.array();
        MatrixXf correspondences_norm = correspondences.array().colwise() - mean2.array();

        MatrixXf W(3,3);
        W(0,0) = (pointcloud_norm.block(0,0,1,pointcloud.cols()) * correspondences_norm.block(0,0,1,correspondences.cols()).transpose())(0);
        W(0,1) = (pointcloud_norm.block(0,0,1,pointcloud.cols()) * correspondences_norm.block(1,0,1,correspondences.cols()).transpose())(0);
        W(0,2) = (pointcloud_norm.block(0,0,1,pointcloud.cols()) * correspondences_norm.block(2,0,1,correspondences.cols()).transpose())(0);

        W(1,0) = (pointcloud_norm.block(1,0,1,pointcloud.cols()) * correspondences_norm.block(0,0,1,correspondences.cols()).transpose())(0);
        W(1,1) = (pointcloud_norm.block(1,0,1,pointcloud.cols()) * correspondences_norm.block(1,0,1,correspondences.cols()).transpose())(0);
        W(1,2) = (pointcloud_norm.block(1,0,1,pointcloud.cols()) * correspondences_norm.block(2,0,1,correspondences.cols()).transpose())(0);

        W(2,0) = (pointcloud_norm.block(2,0,1,pointcloud.cols()) * correspondences_norm.block(0,0,1,correspondences.cols()).transpose())(0);
        W(2,1) = (pointcloud_norm.block(2,0,1,pointcloud.cols()) * correspondences_norm.block(1,0,1,correspondences.cols()).transpose())(0);
        W(2,2) = (pointcloud_norm.block(2,0,1,pointcloud.cols()) * correspondences_norm.block(2,0,1,correspondences.cols()).transpose())(0);



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
        }

        MatrixXf a = R*pointcloud_norm;
        MatrixXf b = correspondences_norm;

        MatrixXf tmp1 = a.cwiseProduct(b);
        MatrixXf tmp2 = a.cwiseProduct(a);

        s = (((float) tmp1.rows())*((float) tmp1.cols())*tmp1.norm()) / (((float) tmp2.rows())*((float) tmp2.cols())*tmp2.norm());

        s = 1.;

        t = mean2 - s*R*mean1;
    }

    void apply_transformation(MatrixXf pointcloud, MatrixXf &pointcloud_proj, MatrixXf R, VectorXf t, float s) {
        pointcloud_proj = s*R*pointcloud;
        pointcloud_proj = pointcloud_proj.array().colwise() + t.array();
    }

float calc_error(MatrixXf source_pointcloud, MatrixXf target_pointcloud, MatrixXf R, VectorXf t, float s) {
        int number_inliers = get_inlier_number(INLIER_PORTION, source_pointcloud.cols());

        MatrixXf source_proj(3, source_pointcloud.cols());
        MatrixXf correspondences(3, source_pointcloud.cols());
        VectorXf distances(source_pointcloud.cols());
        MatrixXf source_proj_trimmed(3, number_inliers);
        MatrixXf correspondences_trimmed(3, number_inliers);

        apply_transformation(source_pointcloud, source_proj, R, t, s);
        find_correspondences(source_proj, target_pointcloud, correspondences, distances);
        trim_pointcloud(source_proj, correspondences, distances, source_proj_trimmed, correspondences_trimmed);

        MatrixXf diff = source_proj_trimmed - correspondences_trimmed;

        float err = 0;
        for (int i = 0; i < diff.cols(); i++) {

            err += sqrt(diff(0,i)*diff(0,i) + diff(1,i)*diff(1,i) + diff(2,i)*diff(2,i));
        }

        return err;
    }

    /*float calc_error(MatrixXf source_pointcloud, MatrixXf target_pointcloud, float inlier_portion) {
        int number_inliers = get_inlier_number(inlier_portion, source_pointcloud.cols());

        MatrixXf correspondences(3, source_pointcloud.cols());
        VectorXf distances(source_pointcloud.cols());
        MatrixXf source_pointcloud_trimmed(3, number_inliers);
        MatrixXf correspondences_trimmed(3, number_inliers);

        find_correspondences(source_pointcloud, target_pointcloud, correspondences, distances);
        trim_pointcloud(source_pointcloud, correspondences, distances, source_pointcloud_trimmed, correspondences_trimmed, number_inliers);
        return calc_error(source_pointcloud_trimmed, correspondences_trimmed);
    }

    float calc_error(MatrixXf source_pointcloud, MatrixXf target_pointcloud) {
        MatrixXf diff = source_pointcloud - target_pointcloud;
        diff = diff.row(0).cwiseProduct(diff.row(0)) + diff.row(1).cwiseProduct(diff.row(1)) + diff.row(2).cwiseProduct(diff.row(2));
        diff = diff.array().sqrt();

        float err = diff.sum();

        if (err < 2) {
            cout<<source_pointcloud<<endl<<target_pointcloud<<endl;
        }

        return diff.sum();
    }*/

    void sort(VectorXf &v) {
      std::sort(v.data(), v.data()+v.size());
    }

    VectorXf matrixToVector(MatrixXf m) {
        m.transposeInPlace();
        VectorXf v(Map<VectorXf>(m.data(), m.cols()*m.rows()));
        return v;
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

    /*boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > random_filter(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pc, int number_points) {
        if (number_points >= pc->size()) {
            return pc;
        }

        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pc_filtered;
        pc_filtered->width = 2000;
        //pc_filtered->width = number_points;
        pc_filtered->height   = 1;pointcloud
        pc_filtered->is_dense = pc->is_dense;
        pc_filtered->points.resize (pc_filtered->width * pc_filtered->height);

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

    }*/

    MatrixXf random_filter(MatrixXf pointcloud, int number_points) {
        cout<<"random filter: "<<pointcloud.cols()<<endl;
        if (pointcloud.cols() <= number_points) {
            return pointcloud;
        }

        vector<int> indices;
        for (int i = 0; i < pointcloud.cols(); i++) {
            indices.push_back(i);
        }
        random_shuffle(indices.begin(), indices.end());

        MatrixXf filtered_pointcloud(pointcloud.rows(), number_points);

        for (int i = 0; i < number_points; i++) {
            filtered_pointcloud(0,i) = pointcloud(0, indices[i]);
            filtered_pointcloud(1,i) = pointcloud(1, indices[i]);
            filtered_pointcloud(2,i) = pointcloud(2, indices[i]);
        }

        return filtered_pointcloud;
    }

    void alignTemplateSrv(MatrixXf R, VectorXf t) {
        ros::ServiceClient align_template_client;
        vigir_object_template_msgs::SetAlignObjectTemplate align_template_srv_;
        ros::NodeHandle nh_;

        geometry_msgs::Quaternion orientation;
        geometry_msgs::Point position;


        align_template_client = nh_.serviceClient<vigir_object_template_msgs::SetAlignObjectTemplate>("/align_object_template");



        position.x = t(0);
        position.y = t(1);
        position.z = t(2);

        orientation.w = sqrt(1. + R(0,0) + R(1,1) + R(2,2)) / 2.;
        orientation.x = (R(2,1) - R(1,2)) / (4.*orientation.w);
        orientation.y = (R(0,2) - R(2,0)) / (4.*orientation.w);
        orientation.z = (R(1,0) - R(0,1)) / (4.*orientation.w);

        //cout<<"align: "<<align_template_srv_.request.pose.pose.position.x<<" "<<align_template_srv_.request.pose.pose.position.y<<" "<<align_template_srv_.request.pose.pose.position.z<<endl;
        //cout<<"align: "<<t(0)<<" "<<t(1)<<" "<<t(2)<<endl;


        align_template_srv_.request.template_id = currentTemplateId; // toDO: id zwischenspeichern

        align_template_srv_.request.pose.pose.position = position;
        align_template_srv_.request.pose.pose.orientation = orientation;

        cout<<"align: "<<align_template_srv_.request.pose.pose.position.x<<" "<<align_template_srv_.request.pose.pose.position.y<<" "<<align_template_srv_.request.pose.pose.position.z<<endl;

        align_template_srv_.request.pose.header.stamp = ros::Time::now();

        if (!align_template_client.call(align_template_srv_))
        {
           ROS_ERROR("Failed to call service request align template");
        }
    }

    int get_inlier_number(float inlier_portion, int number_points) {
        return (int) floor(inlier_portion*((float) number_points));
    }

    MatrixXf getRotationMatrix(float xRot, float yRot, float zRot) {
        MatrixXf R(3,3);

        R << cos(zRot)*cos(yRot), -sin(zRot)*cos(xRot)+cos(zRot)*sin(yRot)*sin(xRot), sin(zRot)*sin(xRot)+cos(zRot)*sin(yRot)*cos(xRot),
             sin(zRot)*cos(yRot),  cos(zRot)*cos(xRot)+sin(zRot)*sin(yRot)*sin(xRot),-cos(zRot)*sin(xRot)+sin(zRot)*sin(yRot)*cos(xRot),
             -sin(yRot),           cos(yRot)*sin(xRot),                               cos(yRot)*cos(xRot);

        return R;
    }
};



int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_alignment");

    PointcloudAlignmentAction pointcloud_alignment(ros::this_node::getName());

    ros::spin();

    return 0;
}

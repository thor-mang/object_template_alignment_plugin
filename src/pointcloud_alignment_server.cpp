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

static int currentTemplateId;


MatrixXf getTemplatePointcloud(string path, string filename);
void saveDataToFile(VectorXf transformation, MatrixXf rotation, MatrixXf template_cloud, MatrixXf world_cloud);

float find_pointcloud_alignment(int number_subclouds, MatrixXf *template_subclouds, MatrixXf world_cloud, float eps, MatrixXf R_init, VectorXf t_init, float s_init, MatrixXf &R_icp, VectorXf &t_icp, float &s_icp,
    int &itCt, int maxIcpIt, float icpEps, int maxDepth, float inlier_portion, float maxTime);
int get_inlier_number(float inlier_portion, int number_points);
float trimmed_scaling_icp(int number_subclouds, MatrixXf *template_subclouds, MatrixXf world_cloud, MatrixXf &R, VectorXf &t, float &s, float eps, int &itCt, int maxIt, float inlier_portion);
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

float noise(float range);
void addNoiseToParams(MatrixXf &R, VectorXf &t, float &s, float range);

float local_icp(MatrixXf pc1, MatrixXf pc2, MatrixXf &R, VectorXf &t, float &s, float eps, int maxIt, float inlier_portion);



static pcl::KdTreeFLANN<pcl::PointXYZ>targetKdTree;


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
        bool success = true;

        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pc_source (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(goal->source_pointcloud , *pc_source);

        MatrixXf source_pointcloud = MatrixXf(3,pc_source->size());

        for (int i = 0; i < pc_source->size(); i++) {
            source_pointcloud(0,i) = pc_source->at(i).x;
            source_pointcloud(1,i) = pc_source->at(i).y;
            source_pointcloud(2,i) = pc_source->at(i).z;
        }


        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pc_target (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(goal->target_pointcloud, *pc_target);

        MatrixXf target_pointcloud = MatrixXf(3,pc_target->size());

        for (int i = 0; i < pc_target->size(); i++) {
            target_pointcloud(0,i) = pc_target->at(i).x;
            target_pointcloud(1,i) = pc_target->at(i).y;
            target_pointcloud(2,i) = pc_target->at(i).z;
        }

        targetKdTree.setInputCloud (pc_target);


        // TODO: in pointcloud alignment verlagern
        int number_subclouds = 5;
        int size_subclouds;

        MatrixXf *source_subclouds = new MatrixXf[number_subclouds];

        for (int i = 0; i < number_subclouds; i++) {
            source_subclouds[i] = random_filter(source_pointcloud, size_subclouds);
        }



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



        int itCt = 0;
        //float error = find_pointcloud_alignment(number_subclouds, source_subclouds, target_pointcloud, 1, R_icp,
        //                                        t_icp, 1., R_icp, t_icp, s_icp, itCt, 300, 1e-7, 1, 0.4, 10);


        float error = trimmed_scaling_icp(number_subclouds, source_subclouds, target_pointcloud, R_icp, t_icp, s_icp, 1e-4, itCt, 300, 0.4);
        cout<<"error: "<<error<<endl;




        // send goal
        if (success) {
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

            cout<<"position server: "<<position.x<<" "<<position.y<<" "<<position.z<<endl;


            // send goal
            geometry_msgs::PoseStamped result;

            result.pose.orientation = orientation;
            result.pose.position = position;

            cout<<"t: "<<goal->initial_pose.pose.position.x<<" "<<goal->initial_pose.pose.position.y<<" "<<goal->initial_pose.pose.position.z<<endl;

            result_.transformation_pose = result;


            ROS_INFO("%s: Succeeded", action_name_.c_str());
            as_.setSucceeded(result_);
        }
    }

    float find_pointcloud_alignment(int number_subclouds, MatrixXf *template_subclouds, MatrixXf world_cloud, float eps, MatrixXf R_init, VectorXf t_init, float s_init, MatrixXf &R_icp,
                                    VectorXf &t_icp, float &s_icp, int &itCt, int maxIcpIt, float icpEps, int maxDepth, float inlier_portion, float maxTime) {

        struct timeval start;
        gettimeofday(&start, NULL);

        float THRESHOLD = 0.013;
        int max_points = 0;

        int queueLength;
        Cube **Q = createPriorityQueue(maxDepth, queueLength);

        volatile bool success = false;

        itCt = 0;
        float minErr = FLT_MAX;
        int icpItCt;

        R_icp = MatrixXf::Identity(3,3);
        t_icp = t_init;
        s_icp = s_init;

        VectorXf t_star(3);
        MatrixXf R_star(3,3);
        t_star << 0.453278, -0.00371148, 0.90559;
        R_star << 0.0155734, -0.999806, 0.0120183, 0.999878, 0.0155607, -0.00114424, 0.00095701, 0.0120346, 0.999927;


        for (int i = 0; i < queueLength; i++) {
            cout<<"percentage: "<< ((float) i) / ((float) queueLength)*100.<<"%, iteration "<<i+1<<"/"<<queueLength << endl;

            if (i != 0 && getPassedTime(start) > maxTime) {
                break;
            }

            MatrixXf R_i = getAARot(Q[i]->r0);
            VectorXf t_i = t_init;
            float s_i = s_init;

            float err = trimmed_scaling_icp(number_subclouds, template_subclouds, world_cloud, R_i, t_i, s_i, 1e-4, icpItCt, 300, 0.4);

            MatrixXf template_proj(3, template_subclouds[0].cols());
            apply_transformation(template_subclouds[0], template_proj, R_i, t_i, s_i);
            int n_points_thres = pointsLowerThanThreshold(template_proj, world_cloud, THRESHOLD);

            float param_err = (R_star-R_i).norm() + (t_star-t_i).norm() + abs(1.-s_i);

            cout<<"err: "<<err<<", param_err: "<<param_err<<", n_points: "<<n_points_thres<<endl;

            //if (param_err < minErr) {
            if (err < minErr && s_i > 0.8) {
            //if (n_points_thres >= max_points) {
                minErr = err;
                max_points = n_points_thres;

                R_icp = R_i;
                t_icp = t_i;
                s_icp = s_i;
            }



            //MatrixXf target_star(3, template_subclouds[0].cols());
            //apply_transformation(template_subclouds[0], target_star, R_star, t_star, 1.);
            //pointsLowerThanThreshold(target_star, world_cloud, 0.1);


        }



        return minErr;
    }

    int get_inlier_number(float inlier_portion, int number_points) {
        return (int) floor(inlier_portion*((float) number_points));
    }

    float trimmed_scaling_icp(int number_subclouds, MatrixXf *template_subclouds, MatrixXf world_cloud, MatrixXf &R, VectorXf &t, float &s, float eps, int &itCt, int maxIt, float inlier_portion) {
        itCt = 0;

        int template_size = template_subclouds[0].cols();
        int number_inliers = get_inlier_number(inlier_portion, template_size);

        MatrixXf cp(3, template_size);
        VectorXf distances(template_size);
        MatrixXf template_proj(3, template_size);
        MatrixXf template_trimmed(3, number_inliers);
        MatrixXf cp_trimmed(3, number_inliers);
        MatrixXf R_old(3,3);
        VectorXf t_old(3);
        float s_old;

        int template_pos = 0;

        MatrixXf template_cloud = template_subclouds[template_pos];

        int numberTries = 0;

        while(itCt < maxIt && numberTries < number_subclouds) {

            template_cloud = template_subclouds[template_pos % number_subclouds];
            itCt++;

            R_old = R;
            t_old = t;
            s_old = s;

            apply_transformation(template_cloud, template_proj, R, t, s);
            find_correspondences(template_proj, world_cloud, cp, distances);
            trim_pc(template_cloud, cp, distances, template_trimmed, cp_trimmed, number_inliers);
            find_transformation(template_trimmed, cp_trimmed, R, t, s);

            if ((R-R_old).norm() + (t-t_old).norm() + abs(s-s_old) < eps) {
                template_pos++;
                numberTries++;
            } else {
                numberTries = 0;
            }
        }

        cout<<"iterations needed: "<<itCt<<" error: "<<calc_error(template_proj, world_cloud, inlier_portion)<<endl;
        return calc_error(template_proj, world_cloud, inlier_portion);
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
        MatrixXf cp(source.rows(), source.cols());
        VectorXf distances(source.cols());
        int number = 0;

        find_correspondences(source, target, cp, distances);

        for (int i = 0; i < source.cols(); i++) {
            if (distances(i) < threshold) {
                number++;
            }
        }

        printDistances(distances);

        return number;
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
                cp(0,i) = pc2(0,pointIdxNKNSearch[0]);
                cp(1,i) = pc2(1,pointIdxNKNSearch[0]);
                cp(2,i) = pc2(2,pointIdxNKNSearch[0]);

                distances(i) = sqrt(pointNKNSquaredDistance[0]);
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

        s = 1.;

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
        if (number_points >= pc->size()) {
            return pc;
        }

        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pc_filtered;
        pc_filtered->width = 2000;
        //pc_filtered->width = number_points;
        pc_filtered->height   = 1;
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

    }

    MatrixXf random_filter(MatrixXf pc, int number_points) {
        cout<<"random filter: "<<pc.cols()<<endl;
        if (number_points >= pc.cols()) {
            return pc;
        }

        cout<<"create indices list"<<endl;
        vector<int> indices;
        for (int i = 0; i < pc.cols(); i++) {
            indices.push_back(i);
        }
        random_shuffle(indices.begin(), indices.end());

        cout<<"create indices list end"<<endl;
        cout<<"pc.rows(): "<<pc.rows()<<endl;
        MatrixXf filtered_pc(pc.rows(), number_points);
        for (int i = 0; i < number_points; i++) {
            cout<<"random filter i: "<<i<<"/"<<number_points-1<<endl;
            filtered_pc(0,i) = pc(0, indices[i]);
            filtered_pc(1,i) = pc(1, indices[i]);
            filtered_pc(2,i) = pc(2, indices[i]);
        }

        return filtered_pc;
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

};



int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_alignment");

    PointcloudAlignmentAction pointcloud_alignment(ros::this_node::getName());

    ros::spin();

    return 0;
}

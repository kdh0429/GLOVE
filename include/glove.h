#include "DataGloveAPI.h"
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "std_msgs/String.h"
#include "math.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Core>


using namespace std;
using namespace Eigen;
class Glove
{
private:
    /* data */
    DataGloveIO* leftGlove;
    DataGloveIO* rightGlove;

    ros::NodeHandle node;
    ros::Publisher allegro_pub;
    ros::Publisher qb_pub;
    ros::Subscriber soft_sensor_sub;
    ros::Publisher allegro_key_pub;
    ros::Subscriber allegro_sub;
    //glove_imu
    ros::Publisher left_glove_imu_pub, right_glove_imu_pub;
    std_msgs::Int16MultiArray left_gloveIMU,right_gloveIMU;
    //end
    float* leftSensors;
    float* rightSensors;
    
	float left_sum;
	float left_avg;
    float rightIndex;
	float rightMiddle;
	float rightRing;
    float rightPinky;

    bool allegro_key_cmd_bool;

	short* leftIMU;
    short* rightIMU; //glove_imu    

	// // classification: find coefficients of shape function for input x
	// float p[3] = { 0.55, 0.91, 0.28 }; // index: weight 0
	// float q[3] = { 0.04, 1.62, 0.23 }; // middle: weight 0.5
	// float r[3] = { 0.04, 0.95, 0.89 }; // ring: weight 0.7
	// // use r as origin
	// float alength = 0.796116;
	// float anorm[3] = { 0.640611, -0.050244, -0.76622 }; //p-r
	// float blength = 0.940479;
	// float bnorm[3] = { 0, 0.712403, -0.70177 }; // q-r
	// float andotbn = 0.5019162;
	// float ancrossbn = 0.864916;
	// float x[3] = { 0.0, 0.0, 0.0 }; // x-r
	// float xdotan, xdotbn  = 0.0;
	// float s, t = 0.0;
	// float ratio_p, ratio_q, ratio_r = 0.0;


	// classification: find coefficients of shape function for input x
	float p[3] = { 0.55, 0.91, 0.28 }; // index: weight 0
	float q[3] = { 0.04, 1.62, 0.23 }; // middle: weight 0.5
	float r[3] = { 0.04, 0.95, 0.89 }; // ring: weight 0.7

    float pqr_inv[9] = { 1.907570, -0.0411672, -0.041791,\
                        -0.848138,   0.745844, -0.758007,\
                        -0.380953,  -0.179795,  1.332630 };

	float x[3];
    float a, b, c;
	float ratio_p, ratio_q, ratio_r;

    //for jacobian
    float x0, x1, x2, x3;
    float x_t0, x_t1, x_t2, x_t3;
    float e0, e1, e2, e3;
    
    //force
    float max_force_x = 150;
    float max_force_y = 50;
    float max_force_z = 150;

    Eigen::MatrixXf index_tip_force;
    float norm_index_tip_force;
    Eigen::MatrixXf middle_tip_force;
    float norm_middle_tip_force;
    Eigen::MatrixXf ring_tip_force;
    float norm_ring_tip_force;
    Eigen::MatrixXf thumb_tip_force;
    float norm_thumb_tip_force;

    //jac, torque for 3 finger
    Eigen::MatrixXf finger_jaco;
    Eigen::MatrixXf finger_joint_torque;
    //jac, torque for thumb
    Eigen::MatrixXf thumb_jaco;
    Eigen::MatrixXf thumb_joint_torque;

    Eigen::MatrixXf t1;
    Eigen::MatrixXf t2;
    Eigen::MatrixXf hmat;
    Eigen::MatrixXf rmat;

    sensor_msgs::JointState allegro_hand_joint_cmd;
	trajectory_msgs::JointTrajectory qb_hand_joint_trajectory;
    std_msgs::String allegro_key_cmd;
    //sensor_msgs::JointState allegro_hand_joint_state;

public:
    Glove(/* args */);
    ~Glove();
    void connectionCheck();
    void calibration();
    void j2j_simple();
    void j2j_thumbrot();
    void linear_mapping();
    float norm(float force_x, float force_y, float force_z);
    float thumb_norm(float force_x, float force_y, float force_z);
    void haptic();
	void gloveLoop(); 
    void setJacobian();
    void handSoftSensorCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void allegroStateSubCallback(const sensor_msgs::JointState::ConstPtr& msg);
   
};


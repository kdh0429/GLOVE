#include "DataGloveAPI.h"
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "std_msgs/String.h"
using namespace std;

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
	// classification: find coefficients of shape function for input x
	float p[3] = { 0.55, 0.91, 0.28 }; // index: weight 0
	float q[3] = { 0.04, 1.62, 0.23 }; // middle: weight 0.5
	float r[3] = { 0.04, 0.95, 0.89 }; // ring: weight 0.7
	// use r as origin
	float alength = 0.796116;
	float anorm[3] = { 0.640611, -0.050244, -0.76622 }; //p-r
	float blength = 0.940479;
	float bnorm[3] = { 0, 0.712403, -0.70177 }; // q-r
	float andotbn = 0.5019162;
	float ancrossbn = 0.864916;
	float x[3] = { 0.0, 0.0, 0.0 }; // x-r
	float xdotan, xdotbn  = 0.0;
	float s, t = 0.0;
	float ratio_p, ratio_q, ratio_r = 0.0;

    sensor_msgs::JointState allegro_hand_joint_cmd;
	trajectory_msgs::JointTrajectory qb_hand_joint_trajectory;
    std_msgs::String allegro_key_cmd;
    //sensor_msgs::JointState allegro_hand_joint_state      ;

public:
    Glove(/* args */);
    ~Glove();
    void connectionCheck();
    void calibration();
	void gloveLoop();
    void handSoftSensorCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void allegroStateSubCallback(const sensor_msgs::JointState::ConstPtr& msg);
};


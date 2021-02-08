#include "glove.h"
Glove::Glove(/* args */)
{
	// 0 : right hand, 1: left hand
	leftGlove = Forte_CreateDataGloveIO(1, "");
	rightGlove = Forte_CreateDataGloveIO(0, "");

	allegro_pub = node.advertise<sensor_msgs::JointState>("allegroHand_0/joint_cmd", 100);	
	qb_pub = node.advertise<trajectory_msgs::JointTrajectory>("qbhand1/control/qbhand1_synergy_trajectory_controller/command", 1000);
	soft_sensor_sub = node.subscribe<std_msgs::Float64MultiArray>("tocabi/handforce", 30, &Glove::handSoftSensorCallback, this);

	leftSensors = Forte_GetSensors(leftGlove);
	rightSensors = Forte_GetSensors(rightGlove);

	connectionCheck();
	calibration();	
}

Glove::~Glove()
{

}


void Glove::connectionCheck()
{
	unsigned char leftConnection = Forte_GetConnectionState(leftGlove);
	unsigned char rightConnection = Forte_GetConnectionState(rightGlove);

	
	cout << "left connection state: " << unsigned(leftConnection) << endl;
	cout << "hand type: " << unsigned(Forte_GetHandType(leftGlove)) << endl;

	cout << "right connection state: " << unsigned(rightConnection) << endl;
	cout << "hand type: " << unsigned(Forte_GetHandType(rightGlove)) << endl;
}

void Glove::calibration()
{
	//calibration
	cout << "press enter to start calibration - right, flat";
	getchar();
	cout << "right, flat - cali start \n";
	Forte_CalibrateFlat(rightGlove);
	cout << "right, flat - cali complete \n";

	cout << "press enter to start calibration - right, Thumb in";
	getchar();
	cout << "right, Thumb in - cali start \n";
	Forte_CalibrateThumbIn(rightGlove);
	cout << "right, Thumb in - cali complete \n";

	cout << "press enter to start calibration - right, Fingers in";
	getchar();
	cout << "right, Fingers in - cali start \n";
	Forte_CalibrateFingersIn(rightGlove);
	cout << "right, Fingers in - cali complete \n";
}

void Glove::handSoftSensorCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	const int sensor_num = 5;
	float threshold = 40.0;
	float sensor_arr[sensor_num];
	for (int i=0; i<sensor_num; i++)
	{
		sensor_arr[sensor_num-1-i] = msg->data[i];
	}
	//haptic feedback
	//actuatorID : Thumb=0, Index=1, Middle=2, Ring=3, Pinky=4, Palm=5
	//note : frequency of vibration, note= 1~127, int
	//amplitude : 0~1, float

	int note=127;
	float amplitude=1.0;

	for (int i=0; i<sensor_num; i++)
	{
		if (sensor_arr[i] > threshold)
		{
			// vibrate i th actuator
			Forte_SendHaptic(leftGlove, i, note, amplitude);
		}
		else{
			Forte_SendHaptic(leftGlove, i, note, 0.0);
		}
	}
}

void Glove::allegroStateSubCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	std::cout << "Right Hand effort: " << msg->effort[1] <<endl;
}



void Glove::gloveLoop()
{
	while (true) {
		// glove sensor index 0~9 (total 10 for each hand)
		// glove Thumb MCP-0, Thumb IP-1, Index MCP-2, Index PIP-3, ...
		// robot hand: 4 for each finger / start from Index finger, inner-0, outter-1,2,3 ...
		// robot hand : last 4 joint for Thumb
		allegro_hand_joint_cmd.position.resize(16);
		allegro_hand_joint_cmd.position[0] = 0.054;
		allegro_hand_joint_cmd.position[1] = 0.097 + rightSensors[2] * 1.5;
		allegro_hand_joint_cmd.position[2] = rightSensors[3] * 1.5;
		allegro_hand_joint_cmd.position[3] = rightSensors[3] * 1.5;

		allegro_hand_joint_cmd.position[4] = -0.138;
		allegro_hand_joint_cmd.position[5] = 0.396 + rightSensors[4] *1.5;
		allegro_hand_joint_cmd.position[6] = rightSensors[5] * 1.5;
		allegro_hand_joint_cmd.position[7] = rightSensors[5] * 1.5;
		
		allegro_hand_joint_cmd.position[8] = -0.521;
		allegro_hand_joint_cmd.position[9] = rightSensors[6] * 1.5;
		allegro_hand_joint_cmd.position[10] = rightSensors[7] * 1.5;
		allegro_hand_joint_cmd.position[11] = rightSensors[7] * 1.5;

		allegro_hand_joint_cmd.position[12] = 1.25; 
		allegro_hand_joint_cmd.position[13] = 0.0; 
		allegro_hand_joint_cmd.position[14] = rightSensors[0] * 1.5;
		allegro_hand_joint_cmd.position[15] = rightSensors[1] * 1.5;

		qb_hand_joint_trajectory.joint_names.resize(1);
		qb_hand_joint_trajectory.joint_names[0] = "qbhand1_synergy_joint";
		qb_hand_joint_trajectory.points.resize(1);
		qb_hand_joint_trajectory.points[0].positions.resize(1);

		left_sum = leftSensors[2]+leftSensors[4]+leftSensors[6]+leftSensors[8];
		left_avg = left_sum/2;
		if (left_avg > 1.0)
			left_avg = 1.0;
		qb_hand_joint_trajectory.points[0].positions[0] = left_avg;
		ros::Duration one_sec(0.3);
		qb_hand_joint_trajectory.points[0].time_from_start = one_sec;

		qb_pub.publish(qb_hand_joint_trajectory);
		ros::spinOnce();
	}
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "GLOVE");
    Glove glove;
    glove.gloveLoop();

    return 0;
}
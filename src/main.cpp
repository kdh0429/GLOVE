#include "glove.h"
Glove::Glove(/* args */)
{
	// 0 : right hand, 1: left hand
	leftGlove = Forte_CreateDataGloveIO(1, "");
	rightGlove = Forte_CreateDataGloveIO(0, "");

	allegro_pub = node.advertise<sensor_msgs::JointState>("allegroHand_0/joint_cmd", 100);	
	qb_pub = node.advertise<trajectory_msgs::JointTrajectory>("qbhand1/control/qbhand1_synergy_trajectory_controller/command", 1000);
	soft_sensor_sub = node.subscribe<std_msgs::Float32MultiArray>("hand_soft_sensor", 1000, &Glove::handSoftSensorCallback, this);

	leftSensors = Forte_GetSensors(leftGlove);
	rightSensors = Forte_GetSensors(rightGlove);

	connectionCheck();
	//calibration();
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

	cout << "press enter to start calibration - right, Tumb in";
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

void Glove::handSoftSensorCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	const int sensor_num = 6;
	float threshold = 0.5;
	float sensor_arr[sensor_num];
	for (int i=0; i<sensor_num; i++)
	{
		sensor_arr[i] = msg->data[i];
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
	}
}


void Glove::gloveLoop()
{
	while (true) {
		x[0] = rightSensors[1] - r[0];
		x[1] = rightSensors[6] - r[1];
		x[2] = rightSensors[10] - r[2];
		xdotan = x[0]*anorm[0] + x[1]*anorm[1] + x[2]*anorm[2];
		xdotbn = x[0]*bnorm[0] + x[1]*bnorm[1] + x[2]*bnorm[2];
		s = (xdotan - andotbn * xdotbn) / (1.0 - andotbn * andotbn);
		t = (xdotbn - andotbn * xdotan) / (1.0 - andotbn * andotbn);
		ratio_p = s/alength;
		ratio_q = t/blength;
		ratio_r = 1.0 - ratio_p - ratio_q;

		// glove sensor index 0~9 (total 10 for each hand)
		// glove Thumb MCP-0, Thumb IP-1, Index MCP-2, Index PIP-3, ...
		// robot hand: 4 for each finger / start from Index finger, inner-0, outter-1,2,3 ...
		// robot hand : last 4 joint for Thumb
		allegro_hand_joint_state.position.resize(16);
		allegro_hand_joint_state.position[0] = 0.054;
		allegro_hand_joint_state.position[1] = 0.097 + rightSensors[2] * 1.5;
		allegro_hand_joint_state.position[2] = rightSensors[3] * 1.5;
		allegro_hand_joint_state.position[3] = rightSensors[3] * 1.5;

		allegro_hand_joint_state.position[4] = -0.138;
		allegro_hand_joint_state.position[5] = 0.396 + rightSensors[4] *1.5;//-0.1 + rightSensors[4] * 1.6;
		allegro_hand_joint_state.position[6] = rightSensors[5] * 1.5;
		allegro_hand_joint_state.position[7] = rightSensors[5] * 1.5;
		
		allegro_hand_joint_state.position[8] = -0.521;
		allegro_hand_joint_state.position[9] = rightSensors[6] * 1.5;
		allegro_hand_joint_state.position[10] = rightSensors[7] * 1.5;
		allegro_hand_joint_state.position[11] = rightSensors[7] * 1.5;

		allegro_hand_joint_state.position[12] = 1.25; //0.1 + rightSensors[0] * 0.2;
		allegro_hand_joint_state.position[13] = 0.0; //0.5 * ratio_q + 0.7 * ratio_r;
		//allegro_hand_joint_state.position[13] = 0.6+rightSensors[0] * 0.2;
		allegro_hand_joint_state.position[14] = rightSensors[0] * 1.5;
		allegro_hand_joint_state.position[15] = rightSensors[1] * 1.5;

		qb_hand_joint_trajectory.joint_names.resize(1);
		qb_hand_joint_trajectory.joint_names[0] = "qbhand1_synergy_joint";
		qb_hand_joint_trajectory.points.resize(1);
		qb_hand_joint_trajectory.points[0].positions.resize(1);

		
		left_sum = leftSensors[2]+leftSensors[4]+leftSensors[6]+leftSensors[8];
		left_avg = left_sum/4;
		qb_hand_joint_trajectory.points[0].positions[0] = left_avg;
		ros::Duration one_sec(1.0);
		qb_hand_joint_trajectory.points[0].time_from_start = one_sec;
		//qb_hand_joint_trajectory.points[0].time_from_start.nsecs = 0;
		/*
		for (int i = 0; i < 10; i++) {
			hand.data[i] = leftSensors[i];
		}
		for (int i = 10; i < 20; i++) {
			hand.data[i] = rightSensors[i-10];
		}
		*/
		allegro_pub.publish(allegro_hand_joint_state);
		qb_pub.publish(qb_hand_joint_trajectory);
		//std::cout << "Right Hand: " << rightSensors[0] << " " << rightSensors[1] << " " << rightSensors[2] << " " << rightSensors[3] << " " << rightSensors[4] << " " << rightSensors[5] << " " << rightSensors[6] << " " << rightSensors[8] << " " << rightSensors[9] << endl;
		//std::cout << "Left Hand: " << qb_hand_joint_trajectory.points[0].positions[0] << endl;
		ros::spinOnce();
	}
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "GLOVE");
    Glove glove;
    glove.gloveLoop();

    return 0;
}
#include "glove.h"
Glove::Glove(/* args */)
{
	// 0 : right hand, 1: left hand
	leftGlove = Forte_CreateDataGloveIO(1, "");
	rightGlove = Forte_CreateDataGloveIO(0, "");

	allegro_pub = node.advertise<sensor_msgs::JointState>("allegroHand_0/joint_cmd", 100);	
	qb_pub = node.advertise<trajectory_msgs::JointTrajectory>("qbhand1/control/qbhand1_synergy_trajectory_controller/command", 1000);
	soft_sensor_sub = node.subscribe<std_msgs::Float32MultiArray>("hand_soft_sensor", 1000, &Glove::handSoftSensorCallback, this);
	allegro_key_pub = node.advertise<std_msgs::String>("allegroHand_0/lib_cmd", 100); //keyboard cmd pub
	allegro_sub = node.subscribe<sensor_msgs::JointState>("allegroHand_0/joint_states", 100, &Glove::allegroStateSubCallback, this);
	//glove_imu
    left_glove_imu_pub = node.advertise<std_msgs::Int16MultiArray>("/left_glove_imu",100);
    right_glove_imu_pub = node.advertise<std_msgs::Int16MultiArray>("/right_glove_imu",100);
    //end
	leftSensors = Forte_GetSensors(leftGlove);
	rightSensors = Forte_GetSensors(rightGlove);

	connectionCheck();
	cout << "connectionCheck() \n";
	calibration();
	cout << "calibration() \n";
	setJacobian();
	cout << "setJacobian() \n";
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

void Glove::setJacobian()
{	
	finger_jaco.resize(3,4);
	thumb_jaco.resize(3,4);
	// for 3 finger of allegro hand
	finger_joint_torque.resize(4,1);
	index_tip_force.resize(3,1);
	middle_tip_force.resize(3,1);
	ring_tip_force.resize(3,1);
	// for thumb
	thumb_joint_torque.resize(4,1);
	thumb_tip_force.resize(3,1);

	//transformation matrix = H_pinv * R
	t1.resize(10,16);
	t1 <<    -0.1499, 0.2614,-0.4733, 0.0548,-0.1161,-0.3676,-0.6599,-0.0978, 0.1132,-0.1581,-0.0242,-0.0809, 0.9641,-1.2068,-0.7681,-0.4185
			,-0.1925,-0.4848,-0.3259,-0.2401, 0.2581, 0.0933, 0.1521, 0.2391, 0.2444, 0.0960, 0.9256, 0.1219, 0.2059,-0.0270, 0.1837, 1.9284
			,-0.7971, 1.0380,-0.5761, 1.5861,-0.1020,-2.4362,-2.0880,-0.2610, 0.6747,-0.2386, 0.8844, 0.8633,-0.3970,-2.6134,-0.0083, 0.0854
			, 0.8292, 0.4456, 2.2121,-0.7257, 0.0386, 2.4378, 2.2545, 0.0924,-0.7722, 0.0221,-0.9340,-1.2236, 0.1002, 2.6221, 0.4366,-0.9622
			, 0.3726, 0.2630, 0.3656,-0.3636,-0.1905, 2.0084, 1.2780, 0.3025,-0.6028, 0.6283,-0.3066,-0.4559, 0.6971, 1.2858, 0.2187,-0.4635
			, 0.0188,-0.3670, 0.9838, 0.1746,-0.2058,-0.2259, 0.7654, 0.1836,-0.0864,-1.0886,-0.2167,-0.1851, 0.2456, 0.0579, 0.2166,-0.8515
			, 0.2072,-0.4247, 0.6175, 0.0246,-0.1384, 0.3480, 0.7512,-0.0512,-0.6929,-0.5129,-0.1550,-0.5413, 1.2463, 0.9108, 0.1758,-0.8366
			,-0.5421, 0.1969,-0.6260, 1.6475, 0.1560,-1.9048,-1.1385, 0.3318, 0.0628, 0.1335, 1.3438, 1.6618, 0.5220,-1.2941, 0.3180, 1.1194
			, 0.0618, 0.5817, 0.2719,-0.7943, 0.3285, 0.6210,-0.0190,-0.1883, 0.0456, 1.4410, 0.4921, 0.4382,-0.0608, 0.4191, 0.4175, 1.0781
			, 0.1884,-0.3027,-0.7587,-0.3933,-0.1197, 0.9004, 0.5458, 0.1263, 0.4496, 0.7562,-0.1459, 0.1452,-1.9025,-0.3942,-0.6078, 0.0005;
	t2.resize(10,16);
	t2 <<    -0.1027, 0.3382,-0.7086,-0.3320,-0.1773, 0.0287,-0.6986,-0.0421, 0.1308,-0.0930,-0.3765,-0.2730, 0.8686,-1.1591,-1.0463,-0.4328
			,-0.2140,-0.2668,-0.3294,-0.1967, 0.2285, 0.0854, 0.0338, 0.2475, 0.2411, 0.2018, 0.8910, 0.2068, 0.1607,-0.1582, 0.1340, 1.8589
			,-0.6260, 1.7663,-1.6273,-0.0545,-0.4263,-0.6903,-2.4703, 0.0013, 0.7467, 0.2384,-0.7413, 0.1610,-0.9014,-2.6344,-1.3321,-0.1015
			, 0.6756,-0.2365, 3.1683, 0.7624, 0.3363, 0.8508, 2.6129,-0.1469,-0.8374,-0.4213, 0.5476,-0.5926, 0.5631, 2.6531, 1.6450,-0.7859
			, 0.3501,-0.3816, 0.7445, 0.1413,-0.0231, 1.4033, 1.6232, 0.1945,-0.6224, 0.2718, 0.3311,-0.3572, 0.9561, 1.5255, 0.7763,-0.2740
			, 0.0401,-0.2552, 0.8435,-0.0411,-0.2510, 0.0062, 0.7065, 0.2192,-0.0770,-1.0179,-0.4356,-0.2729, 0.1753, 0.0462, 0.0369,-0.8812
			, 0.1851,-0.6748, 0.8215, 0.3182,-0.0611, 0.0162, 0.8843,-0.1059,-0.7050,-0.6580, 0.1751,-0.4491, 1.3662, 0.9810, 0.4555,-0.7655
			,-0.4881, 0.9611,-1.1922, 0.8481,-0.0679,-0.9881,-1.5464, 0.4861, 0.0953, 0.5701, 0.4178, 1.4340, 0.1752,-1.5315,-0.4733, 0.8998
			, 0.0987, 0.1775, 0.2913,-0.8526, 0.3861, 0.6137, 0.2002,-0.2068, 0.0508, 1.2434, 0.5748, 0.2930, 0.0274, 0.6569, 0.5240, 1.2064
			, 0.0991,-0.1271,-0.4533, 0.1711,-0.0767, 0.3686, 0.4466, 0.0637, 0.4222, 0.8048, 0.2740, 0.5059,-1.8342,-0.6234,-0.3047,-0.0690;

	hmat.resize(1,10);
	rmat.resize(1,16);
}

void Glove::allegroStateSubCallback(const sensor_msgs::JointState::ConstPtr& msg)
{	
	//int i = 0;
	//std::cout << "Right Hand effort: " << msg->effort[1] <<endl;
	for(int i=0;i<3;i++){
		x0 = msg->position[i*4+0];
		x1 = msg->position[i*4+1];
		x2 = msg->position[i*4+2];
		x3 = msg->position[i*4+3];
		finger_jaco << - 54*sin(x0)*sin(x1) - (297*cos(x3)*(cos(x1)*sin(x0)*sin(x2) + cos(x2)*sin(x0)*sin(x1)))/10 - (297*sin(x3)*(cos(x1)*cos(x2)*sin(x0) - sin(x0)*sin(x1)*sin(x2)))/10 - (192*cos(x1)*sin(x0)*sin(x2))/5 - (192*cos(x2)*sin(x0)*sin(x1))/5, 54*cos(x0)*cos(x1) - (297*cos(x3)*(cos(x0)*sin(x1)*sin(x2) - cos(x0)*cos(x1)*cos(x2)))/10 - (297*sin(x3)*(cos(x0)*cos(x1)*sin(x2) + cos(x0)*cos(x2)*sin(x1)))/10 - (192*cos(x0)*sin(x1)*sin(x2))/5 + (192*cos(x0)*cos(x1)*cos(x2))/5, (192*cos(x0)*cos(x1)*cos(x2))/5 - (297*sin(x3)*(cos(x0)*cos(x1)*sin(x2) + cos(x0)*cos(x2)*sin(x1)))/10 - (192*cos(x0)*sin(x1)*sin(x2))/5 - (297*cos(x3)*(cos(x0)*sin(x1)*sin(x2) - cos(x0)*cos(x1)*cos(x2)))/10, - (297*cos(x3)*(cos(x0)*sin(x1)*sin(x2) - cos(x0)*cos(x1)*cos(x2)))/10 - (297*sin(x3)*(cos(x0)*cos(x1)*sin(x2) + cos(x0)*cos(x2)*sin(x1)))/10,
                54*cos(x0)*sin(x1) + (297*cos(x3)*(cos(x0)*cos(x1)*sin(x2) + cos(x0)*cos(x2)*sin(x1)))/10 - (297*sin(x3)*(cos(x0)*sin(x1)*sin(x2) - cos(x0)*cos(x1)*cos(x2)))/10 + (192*cos(x0)*cos(x1)*sin(x2))/5 + (192*cos(x0)*cos(x2)*sin(x1))/5, 54*cos(x1)*sin(x0) + (297*cos(x3)*(cos(x1)*cos(x2)*sin(x0) - sin(x0)*sin(x1)*sin(x2)))/10 - (297*sin(x3)*(cos(x1)*sin(x0)*sin(x2) + cos(x2)*sin(x0)*sin(x1)))/10 + (192*cos(x1)*cos(x2)*sin(x0))/5 - (192*sin(x0)*sin(x1)*sin(x2))/5, (297*cos(x3)*(cos(x1)*cos(x2)*sin(x0) - sin(x0)*sin(x1)*sin(x2)))/10 - (297*sin(x3)*(cos(x1)*sin(x0)*sin(x2) + cos(x2)*sin(x0)*sin(x1)))/10 + (192*cos(x1)*cos(x2)*sin(x0))/5 - (192*sin(x0)*sin(x1)*sin(x2))/5,   (297*cos(x3)*(cos(x1)*cos(x2)*sin(x0) - sin(x0)*sin(x1)*sin(x2)))/10 - (297*sin(x3)*(cos(x1)*sin(x0)*sin(x2) + cos(x2)*sin(x0)*sin(x1)))/10,
                0,                                                       - 54*sin(x1) - (192*cos(x1)*sin(x2))/5 - (192*cos(x2)*sin(x1))/5 - (297*cos(x3)*(cos(x1)*sin(x2) + cos(x2)*sin(x1)))/10 - (297*sin(x3)*(cos(x1)*cos(x2) - sin(x1)*sin(x2)))/10,                                               - (192*cos(x1)*sin(x2))/5 - (192*cos(x2)*sin(x1))/5 - (297*cos(x3)*(cos(x1)*sin(x2) + cos(x2)*sin(x1)))/10 - (297*sin(x3)*(cos(x1)*cos(x2) - sin(x1)*sin(x2)))/10,                                 - (297*cos(x3)*(cos(x1)*sin(x2) + cos(x2)*sin(x1)))/10 - (297*sin(x3)*(cos(x1)*cos(x2) - sin(x1)*sin(x2)))/10;

		e0 = msg->effort[i*4+0];
		e1 = msg->effort[i*4+1];
		e2 = msg->effort[i*4+2];
		e3 = msg->effort[i*4+3];
		finger_joint_torque << e0, e1, e2, e3;
		if(i==0){
			index_tip_force = finger_jaco * finger_joint_torque;
			
		}
		else if(i==1){
			middle_tip_force = finger_jaco * finger_joint_torque;
			//std::cout << "middle_tip_force: " << middle_tip_force  << std::endl;
			// std::cout << "X: "<<middle_tip_force(0,0) << " / Y: "<< middle_tip_force(1,0) <<" / Z: " << middle_tip_force(2,0) <<endl;
		}
		else if(i==2){
			ring_tip_force = finger_jaco * finger_joint_torque;
		}
	}
	
	////// max_index_tip_force rewrite
	// for (int i=0; i<3; i++)
	// {
	// 	if (max_index_tip_force(i,0)<index_tip_force(i,0))
	// 	{
	// 		max_index_tip_force = index_tip_force(i,0);
	// 	}
	// }
	

	x_t0 = msg->position[12];
	x_t1 = msg->position[13];
	x_t2 = msg->position[14];
	x_t3 = msg->position[15];
	thumb_jaco << (443*sin(x_t3)*(cos(x_t0)*sin(x_t2) - cos(x_t2)*sin(x_t0)*sin(x_t1)))/10 - (257*cos(x_t0)*cos(x_t2))/5 - (443*cos(x_t3)*(cos(x_t0)*cos(x_t2) + sin(x_t0)*sin(x_t1)*sin(x_t2)))/10 - (277*cos(x_t0))/5 - (257*sin(x_t0)*sin(x_t1)*sin(x_t2))/5, (257*cos(x_t0)*cos(x_t1)*sin(x_t2))/5 + (443*cos(x_t0)*cos(x_t1)*cos(x_t2)*sin(x_t3))/10 + (443*cos(x_t0)*cos(x_t1)*cos(x_t3)*sin(x_t2))/10, (257*sin(x_t0)*sin(x_t2))/5 + (443*cos(x_t3)*(sin(x_t0)*sin(x_t2) + cos(x_t0)*cos(x_t2)*sin(x_t1)))/10 + (443*sin(x_t3)*(cos(x_t2)*sin(x_t0) - cos(x_t0)*sin(x_t1)*sin(x_t2)))/10 + (257*cos(x_t0)*cos(x_t2)*sin(x_t1))/5,   (443*cos(x_t3)*(sin(x_t0)*sin(x_t2) + cos(x_t0)*cos(x_t2)*sin(x_t1)))/10 + (443*sin(x_t3)*(cos(x_t2)*sin(x_t0) - cos(x_t0)*sin(x_t1)*sin(x_t2)))/10,
				(443*sin(x_t3)*(sin(x_t0)*sin(x_t2) + cos(x_t0)*cos(x_t2)*sin(x_t1)))/10 - (257*cos(x_t2)*sin(x_t0))/5 - (443*cos(x_t3)*(cos(x_t2)*sin(x_t0) - cos(x_t0)*sin(x_t1)*sin(x_t2)))/10 - (277*sin(x_t0))/5 + (257*cos(x_t0)*sin(x_t1)*sin(x_t2))/5, (257*cos(x_t1)*sin(x_t0)*sin(x_t2))/5 + (443*cos(x_t1)*cos(x_t2)*sin(x_t0)*sin(x_t3))/10 + (443*cos(x_t1)*cos(x_t3)*sin(x_t0)*sin(x_t2))/10, (257*cos(x_t2)*sin(x_t0)*sin(x_t1))/5 - (443*cos(x_t3)*(cos(x_t0)*sin(x_t2) - cos(x_t2)*sin(x_t0)*sin(x_t1)))/10 - (443*sin(x_t3)*(cos(x_t0)*cos(x_t2) + sin(x_t0)*sin(x_t1)*sin(x_t2)))/10 - (257*cos(x_t0)*sin(x_t2))/5, - (443*cos(x_t3)*(cos(x_t0)*sin(x_t2) - cos(x_t2)*sin(x_t0)*sin(x_t1)))/10 - (443*sin(x_t3)*(cos(x_t0)*cos(x_t2) + sin(x_t0)*sin(x_t1)*sin(x_t2)))/10,
                 0,                             - (257*sin(x_t1)*sin(x_t2))/5 - (443*cos(x_t2)*sin(x_t1)*sin(x_t3))/10 - (443*cos(x_t3)*sin(x_t1)*sin(x_t2))/10,                                                                                                             (257*cos(x_t1)*cos(x_t2))/5 - (443*cos(x_t1)*sin(x_t2)*sin(x_t3))/10 + (443*cos(x_t1)*cos(x_t2)*cos(x_t3))/10,                                                                       (443*cos(x_t1)*cos(x_t2)*cos(x_t3))/10 - (443*cos(x_t1)*sin(x_t2)*sin(x_t3))/10;

	e0 = msg->effort[12];
	e1 = msg->effort[13];
	e2 = msg->effort[14];
	e3 = msg->effort[15];	
	thumb_joint_torque << e0, e1, e2, e3;

	thumb_tip_force = thumb_jaco * thumb_joint_torque;
	
	// index_tip_force(4,1);
	// index_tip_force << x0,x1,x2,x3;
	//std::cout << index_tip_force.format(CommaInitFmt) << endl;
	// std::cout << "\r" << "X: "<<index_tip_force(0,0) << " / Y: "<< index_tip_force(1,0) <<" / Z: " << index_tip_force(2,0) <<endl;
	// std::cout << "Index: \t\t\t";
	//std::cout << "\n\n\n" ;
	norm_index_tip_force = norm(index_tip_force(0,0), index_tip_force(1,0), index_tip_force(2,0));
	
	// std::cout << "norm_index_tip_force: " << norm_index_tip_force << std::endl;
	// std::cout << "\n";

	norm_middle_tip_force = norm(middle_tip_force(0,0), middle_tip_force(1,0), middle_tip_force(2,0));
	norm_ring_tip_force = norm(ring_tip_force(0,0), ring_tip_force(1,0), ring_tip_force(2,0));

	// std::cout << "X: "<<thumb_tip_force(0,0) << " / Y: "<< thumb_tip_force(1,0) <<" / Z: " << thumb_tip_force(2,0) <<endl;
	// std::cout << "Thumb: \t\t\t";


	norm_thumb_tip_force = thumb_norm(thumb_tip_force(0,0), thumb_tip_force(1,0), thumb_tip_force(2,0));
	// std::cout << norm_thumb_tip_force  << "\n\n\n" << std::endl;
}

float Glove::norm(float force_x, float force_y, float force_z)
{
	// if (force_z > 0)
	// {force_z = 0;}
	// if (force_x > 0)
	// {force_x = 0;}
	force_y = 0;
	// if (force_y < 0)
	// {force_y = 0;}
	// force_x = 0;
	// force_y = 0;

	//std::cout << "force_x: "<< force_x << " / force_y: "<< force_y  <<" / force_z : " << force_z  <<endl;

	float norm = ((force_x/max_force_x)*(force_x/max_force_x) + (force_y/max_force_y)*(force_y/max_force_y) + (force_z/max_force_z)*(force_z/max_force_z))/3;
	haptic();
	return norm;
}

float Glove::thumb_norm(float force_x, float force_y, float force_z)
{
	if (force_z < 0)
	{force_z = 0;}
	if (force_x > 0)
	{force_x = 0;}
	// if (force_y < 0)
	// {force_y = 0;}
	force_y = 0;
	// force_x = 0;
	// force_y = 0;
	// std::cout << "force_x: "<< force_x << " / force_y: "<< force_y  <<" / force_z : " << force_z  <<endl;
	float norm = ((force_x/max_force_x)*(force_x/max_force_x) + (force_y/max_force_y)*(force_y/max_force_y) + (force_z/max_force_z)*(force_z/max_force_z))/3;
	haptic();
	return norm;
}

void Glove::haptic()
{
	//index
	Forte_SendHaptic(rightGlove, 0, 127, norm_thumb_tip_force*20);
	Forte_SendHaptic(rightGlove, 1, 127, norm_index_tip_force*20);
	Forte_SendHaptic(rightGlove, 2, 127, norm_middle_tip_force*20);
	Forte_SendHaptic(rightGlove, 3, 127, norm_ring_tip_force*20);	
}

void Glove::j2j_simple()
{
	// glove sensor index 0~9 (total 10 for each hand)
	// glove Thumb MCP-0, Thumb IP-1, Index MCP-2, Index PIP-3, ...
	// robot hand: 4 for each finger / start from Index finger, inner-0, outter-1,2,3 ...
	// robot hand : last 4 joint for Thumb

	//index finger
	allegro_hand_joint_cmd.position.resize(16);
	allegro_hand_joint_cmd.position[0] = -0.05;
	allegro_hand_joint_cmd.position[1] = -0.02 +  rightSensors[2] * 1.5;
	allegro_hand_joint_cmd.position[2] = rightSensors[3] * 1.5;
	allegro_hand_joint_cmd.position[3] = rightSensors[3] * 1.5;

	//fix finger - 0218
	// allegro_hand_joint_cmd.position[4] = -0.0115;
	// allegro_hand_joint_cmd.position[5] = 0.4084;
	// allegro_hand_joint_cmd.position[6] = 0.3730;
	// allegro_hand_joint_cmd.position[7] = -0.0357;

	// allegro_hand_joint_cmd.position[8] = -0.5398;
	// allegro_hand_joint_cmd.position[9] = -0.0084;
	// allegro_hand_joint_cmd.position[10] = 0.5422;
	// allegro_hand_joint_cmd.position[11] = -0.1001;

	//middle finger
	allegro_hand_joint_cmd.position[4] = -0.0138;
	allegro_hand_joint_cmd.position[5] = 0.2 + rightSensors[4] *1.5;//-0.1 + rightSensors[4] * 1.6;
	allegro_hand_joint_cmd.position[6] = 0.1+ rightSensors[5] * 1.5;
	allegro_hand_joint_cmd.position[7] = rightSensors[5] * 1.5;
	
	//ring finger
	allegro_hand_joint_cmd.position[8] = -0.521;
	allegro_hand_joint_cmd.position[9] = rightSensors[6] * 1.5;
	allegro_hand_joint_cmd.position[10] = rightSensors[7] * 1.5;
	allegro_hand_joint_cmd.position[11] = rightSensors[7] * 1.5;

	//allegro_hand_joint_cmd.position[12] = 1.25; 
	//allegro_hand_joint_cmd.position[13] = 0.0;
	
	// thumb 
	//rotation
	allegro_hand_joint_cmd.position[12] = 1.25;
	//allegro_hand_joint_cmd.position[13] = 0.6+rightSensors[0] * 0.2;
	
	//allegro_hand_joint_cmd.position[14] =  rightSensors[0] * 1.5;
	allegro_hand_joint_cmd.position[14] =  rightSensors[1] * 1.5;
	allegro_hand_joint_cmd.position[15] =  rightSensors[1] * 1.5;

	//glove joint
	rightIndex = (rightSensors[2]+rightSensors[3])/2;
	rightMiddle = (rightSensors[4]+rightSensors[5])/2;
	rightRing = (rightSensors[6]+rightSensors[7])/2;
	rightPinky = (rightSensors[8]+rightSensors[9])/2;	 
	
	//key cmd

	// if(rightSensors[1]>0.5 && rightIndex>0.5 && rightMiddle<0.35 && rightRing<0.35 && rightPinky<0.35){
	// 	//allegro_key_cmd.data.resize(1);
	// 	allegro_key_cmd.data = "pinch_it";
	// 	if (allegro_key_cmd_bool == false){
	// 		allegro_key_pub.publish(allegro_key_cmd);
	// 		allegro_key_cmd_bool = true;
	// 		std::cout << "pinch_it_pub " << endl;
	// 	}
	// 	//std::cout << "pinch_it " << endl;
	// }
	// else{
	// 	allegro_pub.publish(allegro_hand_joint_cmd);
	// 	allegro_key_cmd_bool = false;
	// }

	allegro_pub.publish(allegro_hand_joint_cmd);
}

void Glove::j2j_thumbrot()
{
	// x[0] = rightSensors[1] - r[0];
	// x[1] = rightSensors[6] - r[1];
	// x[2] = rightSensors[10] - r[2];
	x[0] = rightSensors[2];
	x[1] = rightSensors[4];
	x[2] = rightSensors[6];
	

	a = pqr_inv[0]*x[0]+pqr_inv[1]*x[1]+pqr_inv[2]*x[2];
	b = pqr_inv[3]*x[0]+pqr_inv[4]*x[1]+pqr_inv[5]*x[2];
	c = pqr_inv[6]*x[0]+pqr_inv[7]*x[1]+pqr_inv[8]*x[2];

	ratio_p = a/(a+b+c);
	ratio_q = b/(a+b+c);
	ratio_r = 1.0 - ratio_p - ratio_q;

	// xdotan = x[0]*anorm[0] + x[1]*anorm[1] + x[2]*anorm[2];
	// xdotbn = x[0]*bnorm[0] + x[1]*bnorm[1] + x[2]*bnorm[2];
	// s = (xdotan - andotbn * xdotbn) / (1.0 - andotbn * andotbn);
	// t = (xdotbn - andotbn * xdotan) / (1.0 - andotbn * andotbn);
	// ratio_p = s/alength;
	// ratio_q = t/blength;
	// ratio_r = 1.0 - ratio_p - ratio_q;


	// glove sensor index 0~9 (total 10 for each hand)
	// glove Thumb MCP-0, Thumb IP-1, Index MCP-2, Index PIP-3, ...
	// robot hand: 4 for each finger / start from Index finger, inner-0, outter-1,2,3 ...
	// robot hand : last 4 joint for Thumb

	//index finger
	allegro_hand_joint_cmd.position.resize(16);
	allegro_hand_joint_cmd.position[0] = -0.05;
	allegro_hand_joint_cmd.position[1] = -0.02 +  rightSensors[2] * 1.5;
	allegro_hand_joint_cmd.position[2] = rightSensors[3] * 1.5;
	allegro_hand_joint_cmd.position[3] = rightSensors[3] * 1.5;

	//fix finger - 0218
	// allegro_hand_joint_cmd.position[4] = -0.0115;
	// allegro_hand_joint_cmd.position[5] = 0.4084;
	// allegro_hand_joint_cmd.position[6] = 0.3730;
	// allegro_hand_joint_cmd.position[7] = -0.0357;

	// allegro_hand_joint_cmd.position[8] = -0.5398;
	// allegro_hand_joint_cmd.position[9] = -0.0084;
	// allegro_hand_joint_cmd.position[10] = 0.5422;
	// allegro_hand_joint_cmd.position[11] = -0.1001;

	//middle finger
	allegro_hand_joint_cmd.position[4] = -0.0138;
	allegro_hand_joint_cmd.position[5] = 0.2 + rightSensors[4] *1.5;//-0.1 + rightSensors[4] * 1.6;
	allegro_hand_joint_cmd.position[6] = 0.1+ rightSensors[5] * 1.5;
	allegro_hand_joint_cmd.position[7] = rightSensors[5] * 1.5;
	
	//ring finger
	allegro_hand_joint_cmd.position[8] = -0.521;
	allegro_hand_joint_cmd.position[9] = rightSensors[6] * 1.5;
	allegro_hand_joint_cmd.position[10] = rightSensors[7] * 1.5;
	allegro_hand_joint_cmd.position[11] = rightSensors[7] * 1.5;

	//allegro_hand_joint_cmd.position[12] = 1.25; 
	//allegro_hand_joint_cmd.position[13] = 0.0;
	
	// thumb 
	//rotation
	allegro_hand_joint_cmd.position[12] = 1.25;
	allegro_hand_joint_cmd.position[13] = -0.01 + 0.5 * ratio_q + 0.7 * ratio_r;
	//allegro_hand_joint_cmd.position[13] = 0.6+rightSensors[0] * 0.2;
	
	//allegro_hand_joint_cmd.position[14] =  rightSensors[0] * 1.5;
	allegro_hand_joint_cmd.position[14] =  rightSensors[1] * 1.5;
	allegro_hand_joint_cmd.position[15] =  rightSensors[1] * 1.5;


	allegro_pub.publish(allegro_hand_joint_cmd);

}

void Glove::linear_mapping()
{
	allegro_hand_joint_cmd.position.resize(16);
	//std::cout << "linear mapping" << std::endl;
	// rmat = hmat * t2;
	for(int i=0;i<10;i++)
	{
		hmat(0,i) = rightSensors[i];
	}
	//std::cout << "first for" << std::endl;
	//std::cout << "hmat: " << hmat << std::endl;
	
	//std::cout << "t1: " << t1 << std::endl;
	rmat = hmat * t1;
	//std::cout << "rmat: " << rmat << std::endl;
	for(int j=0;j<16;j++)
	{
		allegro_hand_joint_cmd.position[j] = rmat(0,j);
		// std::cout << "loop2" << std::endl;

	}
	
	std::cout << "second for" << std::endl;
	allegro_pub.publish(allegro_hand_joint_cmd);

	
}

void Glove::gloveLoop()
{
	while (true) 
	{
		//mappping function
		//mapping function should include publisher.

		// j2j_simple();
		j2j_thumbrot(); //여기서 각 함수별로 바꿔가며 작동 정상적으로 하는지를 먼저 봐주세요~ 넹~
		// linear_mapping();


		//qb
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
		qb_pub.publish(qb_hand_joint_trajectory);

		
		//glove_imu
		leftIMU=Forte_GetIMU(leftGlove);  //Output = [rotation left/right, positioning left/right, rotation up/down, positioning up/down] 
		rightIMU=Forte_GetIMU(rightGlove);  //Output = [rotation left/right, positioning left/right, rotation up/down, positioning up/down] 
		left_gloveIMU.data.clear();
		right_gloveIMU.data.clear();
		for(short i=0 ; i<4; i++){
			left_gloveIMU.data.push_back(leftIMU[i]);
			right_gloveIMU.data.push_back(rightIMU[i]);
		}
		left_glove_imu_pub.publish(left_gloveIMU);
		right_glove_imu_pub.publish(right_gloveIMU);

		//std::cout << "Right Hand: " << rightSensors[0] << " " << rightSensors[1] << " " << rightSensors[2] << " " << rightSensors[3] << " " << rightSensors[4] << " " << rightSensors[5] << " " << rightSensors[6] << " " << rightSensors[8] << " " << rightSensors[9] << endl;
		//std::cout << "Left Hand: " << qb_hand_joint_trajectory.points[0].positions[0] << endl;
		//std::cout << "Right Hand: " << rightSensors[6] << " " << rightSensors[7] << " " << endl;
		//std::cout << "Right Hand: " << allegro_hand_joint_cmd.position[9] << " " << allegro_hand_joint_cmd.position[10] << " " << endl;
		//std::cout << "Right Hand: " << rightSensors[1] << " " << rightIndex << " " << rightMiddle  << " " <<  rightRing  << " " << rightPinky <<endl;
		ros::spinOnce();
	}
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "GLOVE");
    Glove glove;
	// Eigen::Matrix2f yee;
	// std::cout << "yee\n";
	// // yee(2,2);
	// // std::cout << "yee(2,2)";	
	// yee << 1,2,
	// 3,4;
	// std::cout << "yee <<1,2,3,4\n";
	// yee.resize(2,3);
	// std::cout << "resize row:" << yee.rows() << "col: " << yee.cols() <<std::endl ;	
    // Eigen::MatrixXf dyn;
	// std::cout << "dyn\n";
	// dyn.resize(1,2);
	// std::cout << "dyn(1,2)\n";
	// dyn << 1.0f, 2.0f;
	// std::cout << "flffjfjfs\n";
	glove.gloveLoop();

    return 0;
}
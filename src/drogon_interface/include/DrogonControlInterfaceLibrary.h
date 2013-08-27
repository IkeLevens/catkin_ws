#include <cstdlib>			//standard library for C/C++
#include <iostream>			//input and output stream packages
#include <string>			//character string class
#include <map>				//data structure to store value by reference key
#include <stdio.h>			//headers for standard input and output
#include <unistd.h>			//_getch*/
#include <termios.h>		//_getch*/
#include <unistd.h>
//#include <roscpp>			//ROS C++ API
#include <sstream>			//For sending topic messages in ROS
#include <ros/ros.h>		//Headers for ros
#include <std_msgs/Header.h>//Headers for ros message classes
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>				//Headers for reading joint positions
#include <baxter_msgs/GripperCommand.h>	//Headers for messages copied from Baxter RSDK
#include <baxter_msgs/JointCommandMode.h>
#include <baxter_msgs/JointPositions.h>
#include <baxter_msgs/JointVelocities.h>
#include <baxter_msgs/EndpointState.h>
#include <baxter_msgs/SolvePositionIK.h>	//Header for service copied from Baxter RSDK
#include <ros/callback_queue.h>
using namespace std;
#ifndef DROGON_INTERFACE_LIBRARY
#define DROGON_INTERFACE_LIBRARY
const int LEFT = 0;
const int RIGHT = 1;
const int OPEN = 2;
const int CLOSE = 3;
const string jointNames [] = {
	"e0",
	"e1",
	"s0",
	"s1",
	"w0",
	"w1",
	"w2"
};
class State
{
	public:
	map<string, double> data;
	void stateCallback (const sensor_msgs::JointState& msg);
};
class Position
{
	public:
	geometry_msgs::Pose pose;
	void positionCallback (const baxter_msgs::EndpointState& msg);
	string toString();
};
class Limits
{
	public:
	map <string, double> jointAngleUpperLimits;
	map <string, double> jointAngleLowerLimits;
	Limits();
};
class DrogonControlInterface
{
	private:
	Limits limits;
	State leftState;
	State rightState;
	Position rightPosition;
	Position leftPosition;
	ros::NodeHandle n;
	ros::Publisher leftArmPub;
	ros::Publisher rightArmPub;
	ros::Publisher leftVelocityPub;
	ros::Publisher rightVelocityPub;
	ros::Publisher leftGripPub;
	ros::Publisher rightGripPub;
	ros::Publisher leftCalibratePub;
	ros::Publisher rightCalibratePub;
	ros::Subscriber leftSub;
	ros::Subscriber rightSub;
	ros::Subscriber leftEndSub;
	ros::Subscriber rightEndSub;
	ros::ServiceClient ikLeftClient;
	ros::ServiceClient ikRightClient;
	void verifyGoalLimits(map<string, double>& goal);
	bool closeEnough (map<string, double>& goal, map<string, double>& state);
	ros::Publisher getArmPublisher(int arm, ros::NodeHandle& n);
	ros::Publisher getCalibratePublisher(int arm, ros::NodeHandle& n);
	ros::Publisher getGripperPublisher(int arm, ros::NodeHandle& n);
	ros::Publisher getVelocityPublisher(int arm);
	ros::Subscriber getArmSubscriber (int arm, State& state, ros::NodeHandle& n);
	ros::Subscriber getEndSubscriber (int arm, Position& position, ros::NodeHandle& n);
	ros::Publisher getVelocityPublisher(int arm, ros::NodeHandle& n);
	ros::ServiceClient getIKServiceClient(int arm, ros::NodeHandle& n);
	public:
	DrogonControlInterface();
	void rosEnable();
	void rosDisable();
	void setJointPosition(int arm, map<string, double> goal);
	void setJointVelocities(int arm, map<string, double> velocities, unsigned int time);
	void gripperAction (int arm, int direction);
	void calibrate (int arm);
	void setPositionMode(int arm);
	void setVelocityMode(int arm);
	map<string, double> getJointStates(int arm);
	bool getIKSolution (int arm, geometry_msgs::Pose pose, map<string, double> &out);
	geometry_msgs::Pose getPose(int arm);
	Position getPosition(int arm);
};

#endif

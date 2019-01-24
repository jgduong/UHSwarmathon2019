#include <ros/ros.h>
#include <math.h>
#include <unordered_map>
#include <set>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// ROS messages
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include "swarmie_msgs/Waypoint.h"

// Include Controllers
#include "LogicController.h"
#include <vector>

#include "Point.h"
#include "Tag.h"

// To handle shutdown signals so the node quits
// properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

#include <math.h>
#include <exception> // For exception handling

using namespace std;
// Define Exceptions
// Define an exception to be thrown if the user tries to create
// a RangeShape using invalid dimensions
class ROSAdapterRangeShapeInvalidTypeException : public std::exception {
public:
  ROSAdapterRangeShapeInvalidTypeException(std::string msg) {
    this->msg = msg;
  }
  
  virtual const char* what() const throw()
  {
    std::string message = "Invalid RangeShape type provided: " + msg;
    return message.c_str();
  }
  
private:
  std::string msg;
};


// Numeric Variables for rover positioning
geometry_msgs::Pose2D currentLocationOdom;		//current location using ODOM
geometry_msgs::Pose2D currentLocationMap;	//current location using GPS
geometry_msgs::Pose2D currentLocationAverage;	//an average of the robots current location

geometry_msgs::Pose2D centerLocationMap;	//A GPS point of the center location, used to help reduce drift from ODOM
geometry_msgs::Pose2D centerLocationOdom;	//The centers location based on ODOM
geometry_msgs::Pose2D centerLocationMapRef;	//Variable used in TransformMapCenterToOdom, can be moved to make it local instead of global


float sonarLeftData = 0.0;
float sonarCenterData = 0.0;
float sonarRightData = 0.0;

bool initialized = false;

geometry_msgs::Twist velocity;

float linearVelocity = 0;	//forward speed, POSITIVE = forward, NEGATIVE = backward
float angularVelocity = 0;	//turning speed, POSITIVE = left, NEGATIVE = right

// Publishers
ros::Publisher stateMachinePublish;		//publishes state machine status
ros::Publisher status_publisher;		//publishes rover status
ros::Publisher fingerAnglePublish;		//publishes gripper angle to move gripper fingers
ros::Publisher wristAnglePublish;		//publishes wrist angle to move wrist
ros::Publisher infoLogPublisher;		//publishes a message to the infolog box on GUI
ros::Publisher driveControlPublish;		//publishes motor commands to the motors
ros::Publisher heartbeatPublisher;		//publishes ROSAdapters status via its "heartbeat"
// Publishes swarmie_msgs::Waypoint messages on "/<robot>/waypooints"
// to indicate when waypoints have been reached.
ros::Publisher waypointFeedbackPublisher;	//publishes a waypoint to travel to if the rover is given a waypoint in manual mode
ros::Publisher robotnames;			//publishes name of robot to /swarmies
ros::Publisher visitedLocationsPublisher;

// Subscribers
ros::Subscriber joySubscriber;			//receives joystick information
ros::Subscriber modeSubscriber; 		//receives mode from GUI
ros::Subscriber targetSubscriber;		//receives tag data
ros::Subscriber odometrySubscriber;		//receives ODOM data
ros::Subscriber mapSubscriber;			//receives GPS data
ros::Subscriber virtualFenceSubscriber;		//receives data for vitrual boundaries
// manualWaypointSubscriber listens on "/<robot>/waypoints/cmd" for
// swarmie_msgs::Waypoint messages.
ros::Subscriber manualWaypointSubscriber; 	//receives manual waypoints given from GUI
ros::Subscriber nameSubscriber;			//testing
ros::Subscriber visitedLocationsSubscriber;

// Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer publish_heartbeat_timer;

void sendDriveCommand(double left, double right);
void humanTime();
void transformMapCentertoOdom();
// records time for delays in sequenced actions, 1 second resolution.
time_t timerStartTime;

// An initial delay to allow the rover to gather enough position data to 
// average its location.
unsigned int startDelayInSeconds = 30;
float timerTimeElapsed = 0;

int currentMode = 0;
const float waypointTolerance = 0.1;

//Transforms
tf::TransformListener *tfListener;

// OS Signal Handler
void sigintEventHandler(int signal);

//Callback handlers
void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message);				//for joystick control
void modeHandler(const std_msgs::UInt8::ConstPtr& message);				//for detecting which mode the robot needs to be in
void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tagInfo);	//receives and stores April Tag Data using the TAG class
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);			//receives and stores ODOM information
void mapHandler(const nav_msgs::Odometry::ConstPtr& message);				//receives and stores GPS information
void virtualFenceHandler(const std_msgs::Float32MultiArray& message);			//Used to set an invisible boundary for robots to keep them from traveling outside specific bounds
void manualWaypointHandler(const swarmie_msgs::Waypoint& message);			//Receives a waypoint (from GUI) and sets the coordinates
void behaviourStateMachine(const ros::TimerEvent&);					//Upper most state machine, calls logic controller to perform all actions
void publishStatusTimerEventHandler(const ros::TimerEvent& event);			//Publishes "ONLINE" when rover is successfully connected
void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);			
void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight);	//handles ultrasound data and stores data

void nameHandler(const std_msgs::String::ConstPtr& message);				//subscriber to sync names of robots together
void visitedLocationsHandler(const std_msgs::Float32MultiArray::ConstPtr& message);

// Converts the time passed as reported by ROS (which takes Gazebo simulation rate into account) into milliseconds as an integer.
long int getROSTimeInMilliSecs();

char host[128];		//rovers hostname
string publishedName;	//published hostname
char prev_state_machine[128];

vector <string> names;
unordered_map<float, set<float>> visitedLocations;	//hashtable to store visited locations

int main(int argc, char **argv) {
  
  gethostname(host, sizeof (host));
  string hostname(host);
  
  if (argc >= 2) {
    publishedName = argv[1];
    cout << "Welcome to the world of tomorrow " << publishedName
         << "!  Behaviour turnDirectionule started." << endl;
  } else {
    publishedName = hostname;
    cout << "No Name Selected. Default is: " << publishedName << endl;
  }
  
  // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
  ros::init(argc, argv, (publishedName + "_BEHAVIOUR"), ros::init_options::NoSigintHandler);
  ros::NodeHandle mNH;
  
  // Register the SIGINT event handler so the node can shutdown properly
  signal(SIGINT, sigintEventHandler);
  
  //subscribers
  joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);					//receives joystick information
  modeSubscriber = mNH.subscribe((publishedName + "/mode"), 1, modeHandler);						//receives mode from GUI
  targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);					//receives tag data
  odometrySubscriber = mNH.subscribe((publishedName + "/odom/filtered"), 10, odometryHandler);				//receives ODOM data
  mapSubscriber = mNH.subscribe((publishedName + "/odom/ekf"), 10, mapHandler);						//receives GPS data

  nameSubscriber = mNH.subscribe(("/swarmies"), 10, nameHandler);							//syncs a list of names

	
  //virtualFenceSubscriber = mNH.subscribe(("/virtualFence"), 10, virtualFenceHandler);					//receives data for vitrual boundaries
  manualWaypointSubscriber = mNH.subscribe((publishedName + "/waypoints/cmd"), 10, manualWaypointHandler);		//receives manual waypoints given from GUI
  message_filters::Subscriber<sensor_msgs::Range> sonarLeftSubscriber(mNH, (publishedName + "/sonarLeft"), 10);
  message_filters::Subscriber<sensor_msgs::Range> sonarCenterSubscriber(mNH, (publishedName + "/sonarCenter"), 10);
  message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(mNH, (publishedName + "/sonarRight"), 10);
  visitedLocationsSubscriber = mNH.subscribe<std_msgs::Float32MultiArray>(("/visitedLocation"), 10, visitedLocationsHandler);
	
  //publishers
  status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);				//publishes rover status
  stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);			//publishes state machine status
  fingerAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/fingerAngle/cmd"), 1, true);			//publishes gripper angle to move gripper finger
  wristAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/wristAngle/cmd"), 1, true);			//publishes wrist angle to move wrist
  infoLogPublisher = mNH.advertise<std_msgs::String>("/infoLog", 1, true);						//publishes a message to the infolog box on GUI
  driveControlPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/driveControl"), 10);			//publishes motor commands to the motors
  heartbeatPublisher = mNH.advertise<std_msgs::String>((publishedName + "/behaviour/heartbeat"), 1, true);		//publishes ROSAdapters status via its "heartbeat"
  waypointFeedbackPublisher = mNH.advertise<swarmie_msgs::Waypoint>((publishedName + "/waypoints"), 1, true);		//publishes a waypoint to travel to if the rover is given a waypoint in manual mode
  visitedLocationsPublisher = mNH.advertise<std_msgs::Float32MultiArray>(("/visitedLocation"), 10, true);
	
  robotnames = mNH.advertise<std_msgs::String>(("/swarmies"), 10, true);						//publishes robotnames to /swarmies

  //timers
  publish_status_timer = mNH.createTimer(ros::Duration(1), publishStatusTimerEventHandler);
  stateMachineTimer = mNH.createTimer(ros::Duration(0.1), behaviourStateMachine);
  
  publish_heartbeat_timer = mNH.createTimer(ros::Duration(2), publishHeartBeatTimerEventHandler);
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;
  
  message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonarLeftSubscriber, sonarCenterSubscriber, sonarRightSubscriber);
  sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));
  
  tfListener = new tf::TransformListener();
  std_msgs::String msg;
  msg.data = "Log Started";
  infoLogPublisher.publish(msg);
  
  stringstream ss;
  ss << "Rover start delay set to " << startDelayInSeconds << " seconds";
  msg.data = ss.str();
  infoLogPublisher.publish(msg);
	/*
	if(currentMode != 2 && currentMode != 3)
	  {
	    // ensure the logic controller starts in the correct mode.
	    logicController.SetModeManual();
	  }
	*/
	  timerStartTime = time(0);
	  
	  ros::spin();
	  
	  return EXIT_SUCCESS;
}

bool initialMove = false;
bool mapTesting = false;
bool rotateBool = false;
float startingTheta = 0.0;
float ninetyRotate = 0.0;

int step = 0;
float initialPositionTrackerX = 0.0;
float initialPositionTrackerY = 0.0;

void behaviourStateMachine(const ros::TimerEvent&)
{
	//cout << "an instance of behaviorStateMachine has run... " << endl;
	timerTimeElapsed = time(0) - timerStartTime;
	
	if (initialMove)
	{
		//float startingTheta = currentLocationOdom.theta;
		ninetyRotate = currentLocationOdom.theta;
		
		float Position1X = 0.0;
		float Position2X = 0.0;
		float Position3X = 0.0;
		float Position4X = 0.0;
		float Position5X = 0.0;
		
		float Position1Y = 0.0;
		float Position2Y = 0.0;
		float Position3Y = 0.0;
		float Position4Y = 0.0;
		float Position5Y = 0.0;

		if (step == 1)
		{
			cout << "First step of driving..." << endl;
			sendDriveCommand(30.0, 30.0);
			Position1X = initialPositionTrackerX;
			Position1Y = initialPositionTrackerY;

			std_msgs::Float32MultiArray initialPop;
			initialPop.layout.dim.push_back(std_msgs::MultiArrayDimension());
			initialPop.layout.dim[0].size = 2;
			initialPop.layout.dim[0].stride = 1;
			initialPop.layout.dim[0].label = "poop";

			initialPop.data.push_back(roundf((currentLocationOdom.x)*10)/10);
			initialPop.data.push_back(roundf((currentLocationOdom.y)*10)/10);

			visitedLocations[initialPop.data[0]].insert(initialPop.data[1]);

			visitedLocationsPublisher.publish(initialPop);
			/*if (abs(currentLocationOdom.x - Position1X > 0.5))
			{
				step = 2;
				Position2X = currentLocationOdom.x;
				Position2Y = currentLocationOdom.y;
			}
			if (abs(currentLocationOdom.y - Position1Y > 0.5))
			{
				step = 2;
				Position2X = currentLocationOdom.x;
				Position2Y = currentLocationOdom.y;
			}*/
			float displacement = sqrt(((currentLocationOdom.x - Position1X)*(currentLocationOdom.x - Position1X)) + ((currentLocationOdom.y - Position1Y)*(currentLocationOdom.y - Position1Y)));
			if (displacement >= 2.0)
			{
				step = 2;
				Position2X = currentLocationOdom.x;
				Position2Y = currentLocationOdom.y;
				startingTheta = currentLocationOdom.theta;
			}
		}
		else if (step == 2)
		{
			cout << "step 2: rotating 90 degrees right..." << endl;
			
			ninetyRotate = currentLocationOdom.theta;
			
			cout << "startingTheta is: " << startingTheta << ", and ninetyRotate is: " << ninetyRotate << endl;
			if (startingTheta <= -3.1)
			{
				startingTheta = 3.12;
			}
			if (abs(ninetyRotate - startingTheta) >= 1.5)
		      {
			    sendDriveCommand(0.0, 0.0); 
			     cout << "done rotating" << endl;
			    step = 3;
			    
		      }
		      else {
			    sendDriveCommand(30.0, -30.0);
		      }
			
		}
		else if (step == 3)
		{
			cout << "Second step of driving..." << endl;
			sendDriveCommand(30.0, 30.0);

			std_msgs::Float32MultiArray initialPopb;
			initialPopb.layout.dim.push_back(std_msgs::MultiArrayDimension());
			initialPopb.layout.dim[0].size = 2;
			initialPopb.layout.dim[0].stride = 1;
			initialPopb.layout.dim[0].label = "poopb";

			initialPopb.data.push_back(roundf((currentLocationOdom.x)*10)/10);
			initialPopb.data.push_back(roundf((currentLocationOdom.y)*10)/10);

			visitedLocations[initialPopb.data[0]].insert(initialPopb.data[1]);

			visitedLocationsPublisher.publish(initialPopb);
			/*if (abs(currentLocationOdom.x - Position2X > 0.5))
			{
				step = 4;
				Position3X = currentLocationOdom.x;
				Position3Y = currentLocationOdom.y;
			}
			if (abs(currentLocationOdom.y - Position2Y > 0.5))
			{
				step = 4;
				Position3X = currentLocationOdom.x;
				Position3Y = currentLocationOdom.y;
			}*/
			
			float displacement = sqrt(((currentLocationOdom.x - Position2X)*(currentLocationOdom.x - Position2X)) + ((currentLocationOdom.y - Position2Y)*(currentLocationOdom.y - Position2Y)));
			if (displacement >= 5.0)
			{
				step = 4;
				Position3X = currentLocationOdom.x;
				Position3Y = currentLocationOdom.y;
				startingTheta = currentLocationOdom.theta;
			}
		}
		else if (step == 4)
		{
			cout << "rotating 90 degrees right..." << endl;

			ninetyRotate = currentLocationOdom.theta;
			if (startingTheta <= -3.1)
			{
				startingTheta = 3.12;
			}
			if (abs(ninetyRotate - startingTheta) >= 1.5)
		      {
			    sendDriveCommand(0.0, 0.0); 
			     cout << "done rotating" << endl;
			    step = 5;
			    
		      }
		      else {
			    sendDriveCommand(30.0, -30.0);
		      }
		}
		else if (step == 5)
		{
			cout << "Third step of driving..." << endl;
			sendDriveCommand(30.0, 30.0);

			std_msgs::Float32MultiArray initialPopc;
			initialPopc.layout.dim.push_back(std_msgs::MultiArrayDimension());
			initialPopc.layout.dim[0].size = 2;
			initialPopc.layout.dim[0].stride = 1;
			initialPopc.layout.dim[0].label = "poopc";

			initialPopc.data.push_back(roundf((currentLocationOdom.x)*10)/10);
			initialPopc.data.push_back(roundf((currentLocationOdom.y)*10)/10);

			visitedLocations[initialPopc.data[0]].insert(initialPopc.data[1]);

			visitedLocationsPublisher.publish(initialPopc);
			/*if (abs(currentLocationOdom.x - Position2X > 0.5))
			{
				step = 4;
				Position3X = currentLocationOdom.x;
				Position3Y = currentLocationOdom.y;
			}
			if (abs(currentLocationOdom.y - Position2Y > 0.5))
			{
				step = 4;
				Position3X = currentLocationOdom.x;
				Position3Y = currentLocationOdom.y;
			}*/
			
			float displacement = sqrt(((currentLocationOdom.x - Position3X)*(currentLocationOdom.x - Position3X)) + ((currentLocationOdom.y - Position3Y)*(currentLocationOdom.y - Position3Y)));
			if (displacement >= 5.0)
			{
				step = 6;
				Position4X = currentLocationOdom.x;
				Position4Y = currentLocationOdom.y;
				startingTheta = currentLocationOdom.theta;
			}
		}
		else if (step == 6)
		{
			cout << "rotating 90 degrees right..." << endl;
	
			ninetyRotate = currentLocationOdom.theta;
			if (startingTheta <= -3.1)
			{
				startingTheta = 3.12;
			}
			if (abs(ninetyRotate - startingTheta) >= 1.5)
		      {
			    sendDriveCommand(0.0, 0.0); 
			     cout << "done rotating" << endl;
			    step = 7;
			    
		      }
		      else {
			    sendDriveCommand(30.0, -30.0);
		      }
		}
		else if (step == 7)
		{
			cout << "Fourth step of driving..." << endl;
			sendDriveCommand(30.0, 30.0);

			std_msgs::Float32MultiArray initialPopd;
			initialPopd.layout.dim.push_back(std_msgs::MultiArrayDimension());
			initialPopd.layout.dim[0].size = 2;
			initialPopd.layout.dim[0].stride = 1;
			initialPopd.layout.dim[0].label = "poopd";

			initialPopd.data.push_back(roundf((currentLocationOdom.x)*10)/10);
			initialPopd.data.push_back(roundf((currentLocationOdom.y)*10)/10);

			visitedLocations[initialPopd.data[0]].insert(initialPopd.data[1]);

			visitedLocationsPublisher.publish(initialPopd);
			/*if (abs(currentLocationOdom.x - Position2X > 0.5))
			{
				step = 4;
				Position3X = currentLocationOdom.x;
				Position3Y = currentLocationOdom.y;
			}
			if (abs(currentLocationOdom.y - Position2Y > 0.5))
			{
				step = 4;
				Position3X = currentLocationOdom.x;
				Position3Y = currentLocationOdom.y;
			}*/
			
			float displacement = sqrt(((currentLocationOdom.x - Position4X)*(currentLocationOdom.x - Position4X)) + ((currentLocationOdom.y - Position4Y)*(currentLocationOdom.y - Position4Y)));
			if (displacement >= 5.0)
			{
				step = 8;
				Position5X = currentLocationOdom.x;
				Position5Y = currentLocationOdom.y;
				startingTheta = currentLocationOdom.theta;
			}
		}
		else if (step == 8)
		{
			cout << "rotating 90 degrees right..." << endl;

			ninetyRotate = currentLocationOdom.theta;
			if (startingTheta <= -3.1)
			{
				startingTheta = 3.12;
			}
			if (abs(ninetyRotate - startingTheta) >= 1.5)
		      {
			    sendDriveCommand(0.0, 0.0); 
			     cout << "done rotating" << endl;
			    step = 9;
			    
		      }
		      else {
			    sendDriveCommand(30.0, -30.0);
		      }
		}
		else if (step == 9)
		{
			cout << "Fifth and last step of driving..." << endl;
			sendDriveCommand(30.0, 30.0);

			std_msgs::Float32MultiArray initialPope;
			initialPope.layout.dim.push_back(std_msgs::MultiArrayDimension());
			initialPope.layout.dim[0].size = 2;
			initialPope.layout.dim[0].stride = 1;
			initialPope.layout.dim[0].label = "poope";

			initialPope.data.push_back(roundf((currentLocationOdom.x)*10)/10);
			initialPope.data.push_back(roundf((currentLocationOdom.y)*10)/10);

			visitedLocations[initialPope.data[0]].insert(initialPope.data[1]);

			visitedLocationsPublisher.publish(initialPope);
			/*if (abs(currentLocationOdom.x - Position2X > 0.5))
			{
				step = 4;
				Position3X = currentLocationOdom.x;
				Position3Y = currentLocationOdom.y;
			}
			if (abs(currentLocationOdom.y - Position2Y > 0.5))
			{
				step = 4;
				Position3X = currentLocationOdom.x;
				Position3Y = currentLocationOdom.y;
			}*/
			
			float displacement = sqrt(((currentLocationOdom.x - Position5X)*(currentLocationOdom.x - Position5X)) + ((currentLocationOdom.y - Position5Y)*(currentLocationOdom.y - Position5Y)));
			if (displacement >= 2.0)
			{
				step = 10;
				startingTheta = currentLocationOdom.theta;

			}
		}
		else if (step == 10)
		{
			cout << "rotating 90 degrees left..." << endl;

			ninetyRotate = currentLocationOdom.theta;
			if (NinetyRotate <= -3.1)
			{
				NinetyRotate = 3.12;
			}
			if (abs(ninetyRotate - startingTheta) >= 1.5)
		      {
			    sendDriveCommand(0.0, 0.0); 
			     cout << "done rotating" << endl;
			sendDriveCommand(0.0, 0.0);
			initialMove = false;
			    
		      }
		      else {
			    sendDriveCommand(-30.0, 30.0);
		      }
		}
		
		
		
	}
	
	
	if (mapTesting)
	{
		//cout << "in mapTesting loop" << endl;
		//sendDriveCommand(30.0, 30.0);
		//cout << "sendDriveCommand done" << endl;
		//cout << "GPS of " << publishedName << " is x = " << currentLocationMap.x << ", y = " << currentLocationMap.y << ", theta = " << currentLocationMap.theta << endl;
		//cout << publishedName << " is at x = " << currentLocationOdom.x << ", y = " << currentLocationOdom.y << ", theta = " << currentLocationOdom.theta << endl;

		std_msgs::Float32MultiArray myCoordinate;
		myCoordinate.layout.dim.push_back(std_msgs::MultiArrayDimension());
		myCoordinate.layout.dim[0].size = 2;
		myCoordinate.layout.dim[0].stride = 1;
		myCoordinate.layout.dim[0].label = "poop";

		//cout << "float32multiarray has been instantiated...\n";
		myCoordinate.data.push_back(roundf((currentLocationOdom.x)*10)/10);
		myCoordinate.data.push_back(roundf((currentLocationOdom.y)*10)/10);
		//myCoordinate.data[0] = roundf((currentLocationOdom.x)*10)/10;
		//myCoordinate.data[1] = roundf((currentLocationOdom.y)*10)/10;
		//cout << "the float32multiarray's info has been normalized...\n";		
		visitedLocations[myCoordinate.data[0]].insert(myCoordinate.data[1]);
		//cout << "insert something something" << endl;
		visitedLocationsPublisher.publish(myCoordinate);

		//cout << "the float32multiarray has been published\n";
		
		//cout << "searching for " << myCoordinate.data[0] << ", " << myCoordinate.data[1] << "... ";
		
		/*if(visitedLocations.find(myCoordinate.data[0]) != visitedLocations.end()) {
			if (visitedLocations[myCoordinate.data[0]].find(myCoordinate.data[1]) != visitedLocations[myCoordinate.data[0]].end()) {
				cout << "Location exists in hashmap" << endl;	
			}
			else { cout << "This y location has not been visited for the specified x location" << endl; }
		}
		else {
			cout << "Location does not exist" << endl;
		}*/
		//CALCULATE X,Y TO CHECK
		std_msgs::Float32MultiArray checkCoord;
		checkCoord.layout.dim.push_back(std_msgs::MultiArrayDimension());
		checkCoord.layout.dim[0].size = 2;
		checkCoord.layout.dim[0].stride = 1;
		checkCoord.layout.dim[0].label = "check";
		float hypot = sqrt((currentLocationOdom.x * currentLocationOdom.x)+(currentLocationOdom.y * currentLocationOdom.y));
		//SUBTRACT a small constant from hypotenuse
		hypot = hypot - 0.2;
		//CALCULATE new x,y
		checkCoord.data.push_back(roundf((hypot*cos(currentLocationOdom.theta))*10)/10);
		checkCoord.data.push_back(roundf((hypot*sin(currentLocationOdom.theta))*10)/10);
		if(visitedLocations.find(checkCoord.data[0]) != visitedLocations.end()) {
			if (visitedLocations[checkCoord.data[0]].find(checkCoord.data[1]) != visitedLocations[checkCoord.data[0]].end()) {
				cout << "location on the right has been visited" << endl;
				sendDriveCommand(30.0, 30.0);
			}
			else { 
				cout << "This y location has not been visited for the specified x location" << endl; 
				sendDriveCommand(30.0, 0.0);
			}
		}
		else {
			cout << "Location on the right has not been visited" << endl;
			sendDriveCommand(30.0, 0.0);
		}
		
	}
	
	if (rotateBool)
	{
		//Rotate to starting position...
	      ninetyRotate = currentLocationOdom.theta;
	      cout << "Current theta is: " << currentLocationOdom.theta << endl;
		
	      std_msgs::Float32MultiArray myCoordinate;
		myCoordinate.layout.dim.push_back(std_msgs::MultiArrayDimension());
		myCoordinate.layout.dim[0].size = 2;
		myCoordinate.layout.dim[0].stride = 1;
		myCoordinate.layout.dim[0].label = "poop";
		
		myCoordinate.data.push_back(roundf((currentLocationOdom.x)*10)/10);
		myCoordinate.data.push_back(roundf((currentLocationOdom.y)*10)/10);
		
		visitedLocations[myCoordinate.data[0]].insert(myCoordinate.data[1]);

		visitedLocationsPublisher.publish(myCoordinate);
	
		
	      if (abs(ninetyRotate - startingTheta) >= 1.5)
	      {
		    sendDriveCommand(0.0, 0.0);
		     rotateBool = false;
		     initialMove = true;
		     //mapTesting = true;
		      step = 1;
			initialPositionTrackerX = currentLocationOdom.x;
			initialPositionTrackerY = currentLocationOdom.y;
		     cout << "done rotating" << endl;
	      }
	      else {
		    sendDriveCommand(-30.0, 30.0);
	      }
	}
	
	if (!initialized)
  	{
		cout << "not initialized detected... " << endl;
    		if (timerTimeElapsed > startDelayInSeconds)
		{

		      cout << "initialization has run..." << endl;
		      initialized = true;

		      startingTheta = currentLocationOdom.theta;

		      cout << "startingTheta is: " << startingTheta << endl;

		      centerLocationOdom.x = currentLocationOdom.x;
		      centerLocationOdom.y = currentLocationOdom.y;
		      //centerLocationOdom.theta = currentLocation.theta;
		      //SET the centerOdom location by passing that variable here

		      centerLocationMap.x = currentLocationMap.x;
		      centerLocationMap.y = currentLocationMap.y;
			  //SET the centerMap location by passing that variable here		

		      //startTime = getROSTimeInMilliSecs();
			rotateBool = true;
		}
    		else
    		{
      			return;
    		}
  	}
	  
	humanTime();
	
		
}

void sendDriveCommand(double left, double right)
{
	velocity.linear.x = left,
    velocity.angular.z = right;
  
	// publish the drive commands
	driveControlPublish.publish(velocity);
}

void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message)
{
	if (message->detections.size() > 0) 
	{
	    vector<Tag> tags;
	
	    for (int i = 0; i < message->detections.size(); i++) 
		{
	      // Package up the ROS AprilTag data into our own type that does not rely on ROS.
	      Tag loc;
	      loc.setID( message->detections[i].id );
	
	      // Pass the position of the AprilTag
	      geometry_msgs::PoseStamped tagPose = message->detections[i].pose;
	      loc.setPosition( make_tuple( tagPose.pose.position.x,
					   tagPose.pose.position.y,
					   tagPose.pose.position.z ) );
	
	      // Pass the orientation of the AprilTag
	      loc.setOrientation( ::boost::math::quaternion<float>( tagPose.pose.orientation.x,
								    tagPose.pose.orientation.y,
								    tagPose.pose.orientation.z,
								    tagPose.pose.orientation.w ) );
	      tags.push_back(loc);
	    }
	    
	    //logicController.SetAprilTags(tags);
	}
}

void modeHandler(const std_msgs::UInt8::ConstPtr& message)
{
	//set mode auto
}

void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight)
{
	//logicController.SetSonarData(sonarLeft->range, sonarCenter->range, sonarRight->range);
	sonarLeftData = sonarLeft->range;
	sonarCenterData = sonarCenter->range;
	sonarRightData = sonarRight->range;
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message)
{
	currentLocationOdom.x = message->pose.pose.position.x;
	currentLocationOdom.y = message->pose.pose.position.y;
	
	//Get theta rotation by converting quaternion orientation to pitch/roll/yaw
  	tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
 	tf::Matrix3x3 m(q);
  	double roll, pitch, yaw;
 	m.getRPY(roll, pitch, yaw);
  	currentLocationOdom.theta = yaw;
  	
  	linearVelocity = message->twist.twist.linear.x;       //HOW do i know these are accurate?
    angularVelocity = message->twist.twist.angular.z;	  //HOW do i know thesea re accurate?
    
    //SET position data readable everywhere(?)
    //SET velocity data readable everywhere(?)
}
	//this just syncs a vector among all robots
void nameHandler(const std_msgs::String::ConstPtr& msg)
{
	cout << "nameHandeler called, message is: " << endl;
	cout << msg->data.c_str() << endl << endl;

	if (std::find(names.begin(), names.end(), msg->data.c_str() ) != names.end())
	{
	 //if duplicate published, do nothing
	}
	else { //add published string to list
		names.push_back(msg->data.c_str());
	}

	// print current size of vector
	for (int i=0; i<names.size(); i++)     
        cout << names[i] << "\n";
        cout << "Size of vector is: " << names.size() << endl;
	
}

void visitedLocationsHandler(const std_msgs::Float32MultiArray::ConstPtr& msg) {
	
	//std_msgs::Float32MultiArray receivedCoordinate[2];
	float x = msg->data[0];
	float y = msg->data[1];
	visitedLocations[x].insert(y);
	//visitedLocations[myCoordinate[0]].insert(myCoordinate[1]);
	//visitedLocationsPublisher.publish(myCoordinate);
}

// Allows a virtual fence to be defined and enabled or disabled through ROS
/*void virtualFenceHandler(const std_msgs::Float32MultiArray& message) 
{
  // Read data from the message array
  // The first element is an integer indicating the shape type
  // 0 = Disable the virtual fence
  // 1 = circle
  // 2 = rectangle
  int shape_type = static_cast<int>(message.data[0]); // Shape type
  
  if (shape_type == 0)
  {
    logicController.setVirtualFenceOff();
  }
  else
  {
    // Elements 2 and 3 are the x and y coordinates of the range center
    Point center;
    center.x = message.data[1]; // Range center x
    center.y = message.data[2]; // Range center y
    
    // If the shape type is "circle" then element 4 is the radius, if rectangle then width
    switch ( shape_type )
    {
    case 1: // Circle
    {
      if ( message.data.size() != 4 ) throw ROSAdapterRangeShapeInvalidTypeException("Wrong number of parameters for circle shape type in ROSAdapter.cpp:virtualFenceHandler()");
      float radius = message.data[3]; 
      logicController.setVirtualFenceOn( new RangeCircle(center, radius) );
      break;
    }
    case 2: // Rectangle 
    {
      if ( message.data.size() != 5 ) throw ROSAdapterRangeShapeInvalidTypeException("Wrong number of parameters for rectangle shape type in ROSAdapter.cpp:virtualFenceHandler()");
      float width = message.data[3]; 
      float height = message.data[4]; 
      logicController.setVirtualFenceOn( new RangeRectangle(center, width, height) );
      break;
    }
    default:
    { // Unknown shape type specified
      throw ROSAdapterRangeShapeInvalidTypeException("Unknown Shape type in ROSAdapter.cpp:virtualFenceHandler()");
    }
    }
  }
}*/

void mapHandler(const nav_msgs::Odometry::ConstPtr& message)
{
  //Get (x,y) location directly from pose
  currentLocationMap.x = message->pose.pose.position.x;
  currentLocationMap.y = message->pose.pose.position.y;
  
  //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
  tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  currentLocationMap.theta = yaw;
  
  linearVelocity = message->twist.twist.linear.x;
  angularVelocity = message->twist.twist.angular.z;
    //SET position data readable everywhere(?)
    //SET velocity data readable everywhere(?)
}

void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message)
{
	//manual mode not needed
}

void publishStatusTimerEventHandler(const ros::TimerEvent&)
{
  std_msgs::String msg;
  msg.data = "UH";		//change this with team name
  status_publisher.publish(msg);

  msg.data = publishedName;
  //robotnames.publish(msg);
}

void manualWaypointHandler(const swarmie_msgs::Waypoint& message)
{
	//receving waypoints from GUI, not needed yet
}

void sigintEventHandler(int sig) 
{
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

void publishHeartBeatTimerEventHandler(const ros::TimerEvent&)
{
  std_msgs::String msg;
  msg.data = "";
  heartbeatPublisher.publish(msg);
  //names.push_back("msg");
  //cout << "Size of vector is: " << names.size() << endl;
}

long int getROSTimeInMilliSecs()
{
  // Get the current time according to ROS (will be zero for simulated clock until the first time message is recieved).
  ros::Time t = ros::Time::now();
  
  // Convert from seconds and nanoseconds to milliseconds.
  return t.sec*1e3 + t.nsec/1e6;
  
}

Point updateCenterLocation()			//PROBABLY not needed
{
  transformMapCentertoOdom();
  
  Point tmp;
  tmp.x = centerLocationOdom.x;
  tmp.y = centerLocationOdom.y;
  
  return tmp;
}

void transformMapCentertoOdom()
{
  
  // map frame
  geometry_msgs::PoseStamped mapPose;
  
  // setup msg to represent the center location in map frame
  mapPose.header.stamp = ros::Time::now();
  
  mapPose.header.frame_id = publishedName + "/map";
  mapPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, centerLocationMap.theta);
  mapPose.pose.position.x = centerLocationMap.x;
  mapPose.pose.position.y = centerLocationMap.y;
  geometry_msgs::PoseStamped odomPose;
  string x = "";
  
  try
  { //attempt to get the transform of the center point in map frame to odom frame.
    tfListener->waitForTransform(publishedName + "/map", publishedName + "/odom", ros::Time::now(), ros::Duration(1.0));
    tfListener->transformPose(publishedName + "/odom", mapPose, odomPose);
  }
  
  catch(tf::TransformException& ex) {  //bad transform
    ROS_INFO("Received an exception trying to transform a point from \"map\" to \"odom\": %s", ex.what());
    x = "Exception thrown " + (string)ex.what();
    std_msgs::String msg;
    stringstream ss;
    ss << "Exception in mapAverage() " + (string)ex.what();
    msg.data = ss.str();
    infoLogPublisher.publish(msg);
    cout << msg.data << endl;
  }
  
  // Use the position and orientation provided by the ros transform.
  centerLocationMapRef.x = odomPose.pose.position.x; //set centerLocation in odom frame
  centerLocationMapRef.y = odomPose.pose.position.y;
  
 // cout << "x ref : "<< centerLocationMapRef.x << " y ref : " << centerLocationMapRef.y << endl;
  
  float xdiff = centerLocationMapRef.x - centerLocationOdom.x;	//get difference in X values
  float ydiff = centerLocationMapRef.y - centerLocationOdom.y;	//get difference in Y values
  
  float diff = hypot(xdiff, ydiff);	//get total difference
  
  if (diff > 0.1)	//If the difference is greater than tolerance, adjust the rovers perceived idea of where the center is. Used to decrease ODOM drift and keep rover accuracy for longer periods of time
  {
    centerLocationOdom.x += xdiff/diff;	//adjust X
    centerLocationOdom.y += ydiff/diff;	//adjust Y
  }
  
  //cout << "center x diff : " << centerLocationMapRef.x - centerLocationOdom.x << " center y diff : " << centerLocationMapRef.y - centerLocationOdom.y << endl;
  //cout << hypot(centerLocationMapRef.x - centerLocationOdom.x, centerLocationMapRef.y - centerLocationOdom.y) << endl;
          
}
float startTime = 0;
float minutesTime = 0;
float hoursTime = 0;
void humanTime() 
{
  
  float timeDiff = (getROSTimeInMilliSecs()-startTime)/1e3;
  if (timeDiff >= 60) {
    minutesTime++;
    startTime += 60  * 1e3;
    if (minutesTime >= 60) {
      hoursTime++;
      minutesTime -= 60;
    }
  }
  timeDiff = floor(timeDiff*10)/10;
  
  double intP, frac;
  frac = modf(timeDiff, &intP);
  timeDiff -= frac;
  frac = round(frac*10);
  if (frac > 9) {
    frac = 0;
  }
  
  //cout << "System has been Running for :: " << hoursTime << " : hours " << minutesTime << " : minutes " << timeDiff << "." << frac << " : seconds" << endl; //you can remove or comment this out it just gives indication something is happening to the log file
}

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
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include "swarmie_msgs/Waypoint.h"
#include <visualization_msgs/Marker.h>

// Include Controllers
//#include "LogicController.h"
#include <vector>

#include "Point.h"
#include "Tag.h"

// To handle shutdown signals so the node quits
// properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

#include <math.h>
#include <exception> // For exception handling   robertphu sux

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

uint32_t shape = visualization_msgs::Marker::CUBE;
int swarmieid = 0;
int markerid = 0;
bool swarmieidset = false;

template < typename T>
int findInVector(const std::vector<T>  & vecOfElements, const T  & element)
{
	int result;
 
	// Find given element in vector
	auto it = std::find(vecOfElements.begin(), vecOfElements.end(), element);
 
	if (it != vecOfElements.end())
	{
		result = distance(vecOfElements.begin(), it);
	}
	else
	{
		result = -1;
	}
 
	return result;
}

bool initialized = false;


vector <string> names;
geometry_msgs::Twist velocity;
vector<Tag> tags;
int tagIndex = -1;

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
ros::Publisher robotnamePublisher;			//publishes name of robot to /swarmies
ros::Publisher visitedLocationsPublisher;

ros::Publisher marker_pub;

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

ros::Timer mapTestingTimer;

void sendDriveCommand(double left, double right);
void humanTime();
void transformMapCentertoOdom();
float normalizedValue(float x);
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

void nameHandler(const geometry_msgs::Point::ConstPtr& message);				//subscriber to sync names of robots together
void visitedLocationsHandler(const std_msgs::Float32MultiArray::ConstPtr& message);
void spiralSearch(const ros::TimerEvent&);

// Converts the time passed as reported by ROS (which takes Gazebo simulation rate into account) into milliseconds as an integer.
long int getROSTimeInMilliSecs();

char host[128];		//rovers hostname
string publishedName;	//published hostname
char prev_state_machine[128];

vector <geometry_msgs::Point> startingLocationGPS;
unordered_map<float, set<float>> visitedLocations;	//hashtable to store visited locations

bool isVisited(float x, float y) {
	if (visitedLocations.find(x) != visitedLocations.end()) {
		//x location exists in hashmap, check y coordinate
		if (visitedLocations[x].find(y) != visitedLocations[x].end()) {
			//y location also exists, so this coordinate has been visited
			return true;
		}
		else {
			return false;	
		}
	}
	else {
		return false;
	}
}

float calcDistance(float curX, float curY, float goalX, float goalY) {
	float dist = sqrt( (goalX - curX)*(goalX - curX) + (goalY - curY)*(goalY - curY) );
	return dist;
}

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
	
  robotnamePublisher = mNH.advertise<std_msgs::String>(("/swarmies"), 10, true);						//publishes robotnames to /swarmies						//publishes robotnames to /swarmies
  marker_pub = mNH.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

  //timers
  publish_status_timer = mNH.createTimer(ros::Duration(1), publishStatusTimerEventHandler);
  stateMachineTimer = mNH.createTimer(ros::Duration(0.1), behaviourStateMachine);
  mapTestingTimer = mNH.createTimer(ros::Duration(0.05), spiralSearch);
  
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
bool GPSCenter = false;
bool hardcodedPop = false;

bool aprilTagDetected = false;
bool aprilTagAcquireSequence = false;
bool returnToHome = false;
bool rotateToHome = false;
bool driveToHome = false;

float startingTheta = 0.0;
float ninetyRotate = 0.0;

float centerOffsetX = 0.0;
float centerOffsetY = 0.0;

int step = 0;
float initialPositionTrackerX = 0.0;
float initialPositionTrackerY = 0.0;
float Position1X = 0.0;
float Position2X = 0.0;
float Position3X = 0.0;
float Position4X = 0.0;
float Position5X = 0.0;
float Position6X = 0.0;
		
float Position1Y = 0.0;
float Position2Y = 0.0;
float Position3Y = 0.0;
float Position4Y = 0.0;
float Position5Y = 0.0;
float Position6Y = 0.0;

float zDistanceToCube = 0.0;
float tagPickupTimer = 0.0;
int quadrant = 0;
float homeTheta = 0.0;
float initialThetaBeforeHome = 0.0;
float distanceToHome = 0.0;
float startPosX = 0.0;
float startPosY = 0.0;

void behaviourStateMachine(const ros::TimerEvent&)
{
	//cout << "an instance of behaviorStateMachine has run... " << endl;
	timerTimeElapsed = time(0) - timerStartTime;
	
	if (driveToHome)
	{
		//cout << "starting position X, Y is: " << startPosX << ",  " << startPosY << endl;
		//cout << "current location odometry x,y: " << currentLocationOdom.x + centerOffsetX << ", " << currentLocationOdom.y + centerOffsetY << endl;
		cout << "desired distance is: " << distanceToHome << endl;
		sendDriveCommand(60.0, 60.0);
		float displacement = calcDistance((startPosX),(startPosY),(currentLocationOdom.x + centerOffsetX),(currentLocationOdom.y + centerOffsetY));
		cout << "using jenb's formula, displacement is: " << displacement << endl;
		if (abs(displacement - distanceToHome) <= 0.01 || displacement >= distanceToHome)
		{
			sendDriveCommand(0.0, 0.0);
			driveToHome = false;
		}
	}
	if (rotateToHome)
	{
		cout << "initialThetaBeforeHome is: " << initialThetaBeforeHome << endl;
		float turnSize = homeTheta - initialThetaBeforeHome;
		cout << "turnSize here is: " << turnSize << endl;
		bool exceedMag = false;
		ninetyRotate = currentLocationOdom.theta;
		
		if (turnSize > 0.0) // left
		{
			/*if (abs(initialThetaBeforeHome + turnSize) >= 3.142)
			{
				exceedMag = true;
			}
			if (exceedMag)
			{
				float desiredTheta = -3.142 + (initialThetaBeforeHome - turnSize);
				cout << "desired theta calculated as: " << desiredTheta << endl;
				
				if (currentLocationOdom.theta >= desiredTheta && currentLocationOdom.theta < 0.0)
				{
					sendDriveCommand(0.0, 0.0);
					cout << "done rotating " << endl;
					rotateToHome = false;
				}
				else {
					sendDriveCommand(-30.0, 30.0);
				}
			}
			else {
				if (abs(ninetyRotate - initialThetaBeforeHome) >= turnSize)
				{
					sendDriveCommand(0.0, 0.0);
					cout << "done rotating" << endl;
					rotateToHome = false;
				}
				else {
					sendDriveCommand(-30.0, 30.0);
				}
			}*/
			if (abs(currentLocationOdom.theta - homeTheta) <= 0.05)
			{
				sendDriveCommand(0.0, 0.0);
				cout << "done rotating" << endl;
				rotateToHome = false;
				driveToHome = true;
				startPosX = currentLocationOdom.x + centerOffsetX;
				startPosY = currentLocationOdom.y + centerOffsetY;

				/*if (quadrant == 1)
				{
					distanceToHome = sqrt((0.25 - currentLocationOdom.y + centerOffsetY)*(0.25 - currentLocationOdom.y + centerOffsetY) + (0.25 - currentLocationOdom.x + centerOffsetX)*(0.25 - currentLocationOdom.x + centerOffsetX));
				}
				else if (quadrant == 2)
				{
					distanceToHome = sqrt((0.25 - currentLocationOdom.y + centerOffsetY)*(0.25 - currentLocationOdom.y + centerOffsetY) + (-0.25 - currentLocationOdom.x + centerOffsetX)*(-0.25 - currentLocationOdom.x + centerOffsetX));
				}
				else if (quadrant == 3)
				{
					distanceToHome = sqrt((-0.25 - currentLocationOdom.y + centerOffsetY)*(-0.25 - currentLocationOdom.y + centerOffsetY) + (-0.25 - currentLocationOdom.x + centerOffsetX)*(-0.25 - currentLocationOdom.x + centerOffsetX));
				}
				else if (quadrant == 4)
				{
					distanceToHome = sqrt((-0.25 - currentLocationOdom.y + centerOffsetY)*(-0.25 - currentLocationOdom.y + centerOffsetY) + (0.25 - currentLocationOdom.x + centerOffsetX)*(0.25 - currentLocationOdom.x + centerOffsetX));
				}
				else {
					distanceToHome = sqrt((0 - currentLocationOdom.y + centerOffsetY)*(0 - currentLocationOdom.y + centerOffsetY) + (0 - currentLocationOdom.x + centerOffsetX)*(0 - currentLocationOdom.x + centerOffsetX));
				}*/
				distanceToHome = sqrt((0 - currentLocationOdom.y + centerOffsetY)*(0 - currentLocationOdom.y + centerOffsetY) + (0 - currentLocationOdom.x + centerOffsetX)*(0 - currentLocationOdom.x + centerOffsetX));
			}
			else {
				sendDriveCommand(-30.0, 30.0);
			}
		}
		else if (turnSize < 0.0) // right
		{
			/*if (abs(startingTheta + turnSize) >= 3.142)
			{
				exceedMag = true;
			}
			if (exceedMag)
			{
				float desiredTheta = 3.142 + (initialThetaBeforeHome - turnSize);
				cout << "desired theta calculated as: " << desiredTheta << endl;
				
				if (currentLocationOdom.theta <= desiredTheta && currentLocationOdom.theta > 0.0)
				{
					sendDriveCommand(0.0, 0.0);
					cout << "done rotating " << endl;
					rotateToHome = false;
				}
				else {
					sendDriveCommand(30.0, -30.0);
				}
			}
			else {
				if (abs(ninetyRotate - initialThetaBeforeHome) <= turnSize)
				{
					sendDriveCommand(0.0, 0.0);
					cout << "done rotating" << endl;
					rotateToHome = false;
				}
				else {
					sendDriveCommand(30.0, -30.0);
				}
			}*/
			if (abs(currentLocationOdom.theta - homeTheta) <= 0.05)
			{
				sendDriveCommand(0.0, 0.0);
				cout << "done rotating" << endl;
				rotateToHome = false;
				driveToHome = true;
				startPosX = currentLocationOdom.x + centerOffsetX;
				startPosY = currentLocationOdom.y + centerOffsetY;
				/*if (quadrant == 1)
				{
					distanceToHome = sqrt((0.25 - currentLocationOdom.y + centerOffsetY)*(0.25 - currentLocationOdom.y + centerOffsetY) + (0.25 - currentLocationOdom.x + centerOffsetX)*(0.25 - currentLocationOdom.x + centerOffsetX));
				}
				else if (quadrant == 2)
				{
					distanceToHome = sqrt((0.25 - currentLocationOdom.y + centerOffsetY)*(0.25 - currentLocationOdom.y + centerOffsetY) + (-0.25 - currentLocationOdom.x + centerOffsetX)*(-0.25 - currentLocationOdom.x + centerOffsetX));
				}
				else if (quadrant == 3)
				{
					distanceToHome = sqrt((-0.25 - currentLocationOdom.y + centerOffsetY)*(-0.25 - currentLocationOdom.y + centerOffsetY) + (-0.25 - currentLocationOdom.x + centerOffsetX)*(-0.25 - currentLocationOdom.x + centerOffsetX));
				}
				else if (quadrant == 4)
				{
					distanceToHome = sqrt((-0.25 - currentLocationOdom.y + centerOffsetY)*(-0.25 - currentLocationOdom.y + centerOffsetY) + (0.25 - currentLocationOdom.x + centerOffsetX)*(0.25 - currentLocationOdom.x + centerOffsetX));
				}
				else {
					distanceToHome = sqrt((0 - currentLocationOdom.y + centerOffsetY)*(0 - currentLocationOdom.y + centerOffsetY) + (0 - currentLocationOdom.x + centerOffsetX)*(0 - currentLocationOdom.x + centerOffsetX));
				}*/
				distanceToHome = sqrt((0 - currentLocationOdom.y + centerOffsetY)*(0 - currentLocationOdom.y + centerOffsetY) + (0 - currentLocationOdom.x + centerOffsetX)*(0 - currentLocationOdom.x + centerOffsetX));
			}
			else {
				sendDriveCommand(30.0, -30.0);
			}
		}
	}
	
	if (returnToHome)
	{
		
		/*if (currentLocationOdom.x + centerOffsetX > 0.0 && currentLocationOdom.y + centerOffsetY > 0)
		{
			quadrant = 1;
			float theTheta = atan2((0.4 - (currentLocationOdom.y+centerOffsetY)),(0.4 - (currentLocationOdom.x+centerOffsetX)));
			float desiredTheta = theTheta - M_PI;
			cout << "quadrant is: " << quadrant << ", desiredTheta is: " << desiredTheta << endl;
			homeTheta = desiredTheta;
		}
		else if (currentLocationOdom.x + centerOffsetX < 0.0 && currentLocationOdom.y + centerOffsetY > 0)
		{
			quadrant = 2;
			float theTheta = atan2((0.4 - (currentLocationOdom.y+centerOffsetY)),(-0.4 - (currentLocationOdom.x+centerOffsetX)));
			float desiredTheta = -theTheta;
			cout << "quadrant is: " << quadrant << ", desiredTheta is: " << desiredTheta << endl;
			homeTheta = desiredTheta;
		}
		else if (currentLocationOdom.x + centerOffsetX < 0.0 && currentLocationOdom.y + centerOffsetY < 0)
		{
			quadrant = 3;
			float theTheta = atan2((-0.4 - (currentLocationOdom.y+centerOffsetY)),(-0.4 - (currentLocationOdom.x+centerOffsetX)));
			float desiredTheta = theTheta;
			cout << "quadrant is: " << quadrant << ", desiredTheta is: " << desiredTheta << endl;
			homeTheta = desiredTheta;
		}
		else if (currentLocationOdom.x + centerOffsetX > 0.0 && currentLocationOdom.y + centerOffsetY < 0)
		{
			quadrant = 4;
			float theTheta = atan2((-0.4 - (currentLocationOdom.y+centerOffsetY)),(0.4 - (currentLocationOdom.x+centerOffsetX)));
			float desiredTheta = M_PI - theTheta;
			cout << "quadrant is: " << quadrant << ", desiredTheta is: " << desiredTheta << endl;
			homeTheta = desiredTheta;
		}
		else {
		}*/
		homeTheta = atan2((0 - (currentLocationOdom.y + centerOffsetY)),(0 - (currentLocationOdom.x + centerOffsetX)));
		initialThetaBeforeHome = currentLocationOdom.theta;
		returnToHome = false;
		rotateToHome = true;
	}
	
	if (aprilTagAcquireSequence)
	{
		tagPickupTimer++;
		std_msgs::Float32 fngr;
		fngr.data = M_PI_2;
		std_msgs::Float32 wrist;
		wrist.data = 1.25;
		fingerAnglePublish.publish(fngr);
		wristAnglePublish.publish(wrist);
		sendDriveCommand(20.0, 20.0);
		
		if (tagPickupTimer > (zDistanceToCube*20*10))
		{
			sendDriveCommand(0.0, 0.0);
			fngr.data = 0;
			wrist.data = 0;
			fingerAnglePublish.publish(fngr);
			//wristAnglePublish.publish(wrist);
			if (tagPickupTimer > zDistanceToCube*20*10 +30)
			{
				wristAnglePublish.publish(fngr);
				
				if (tagPickupTimer > zDistanceToCube*200 +50)
				{
					aprilTagAcquireSequence = false;
					//mapTesting = true;
					returnToHome = true;
				}
			}
			
			
			
		}
	}
	
	if (aprilTagDetected)
	{
		mapTesting = false;
		//sendDriveCommand(0.0, 0.0);
		
		//tuple<float, float, float> pos = tags[tagIndex].getPosition();
		//float r = get<0>(pos);
		//float p = get<1>(pos);
		//float y = get<2>(pos);
		float x = tags.back().getPositionX();
		float y = tags.back().getPositionY();
		float z = tags.back().getPositionZ();
		
		cout << "x, y, z of aprilTag: " << x << ", " << y << ", " << z << endl;
		
		if ( x > 0.001 )
		{
			sendDriveCommand(5.0, -5.0);
		}
		else if ( x < -0.001 )
		{
			sendDriveCommand(-5.0, 5.0);
		}
		else
		{
			cout << "centered on cube" << endl;
			sendDriveCommand(0.0, 0.0);
			aprilTagAcquireSequence = true;
			zDistanceToCube = z;
			aprilTagDetected = false;
			tagPickupTimer = 0.0;
		}
	}
	
	if (hardcodedPop)
	{
		std_msgs::Float32MultiArray initialPop;
		initialPop.layout.dim.push_back(std_msgs::MultiArrayDimension());
		initialPop.layout.dim[0].size = 2;
		initialPop.layout.dim[0].stride = 1;
		initialPop.layout.dim[0].label = "initialize";	
		
		float x = -1.25;
		float y = -1.25;
		for (x = -1.50; x != 1.75; x+=0.25)
		{
			for (y = -1.50; y != 1.75; y += 0.25)
			{
				initialPop.data.push_back(x);
				initialPop.data.push_back(y);
				
				visitedLocations[initialPop.data[0]].insert(initialPop.data[1]);
				visitedLocationsPublisher.publish(initialPop);
				initialPop.data.clear();
			}
		}
		hardcodedPop = false;
		//mapTesting = true;
		//rotateBool = true;
		initialMove = true;
		step = 10;
		
		startingTheta = currentLocationOdom.theta;
		Position6X = currentLocationOdom.x;
		Position6Y = currentLocationOdom.y;
	}
	
	if (initialMove)
	{
		//float startingTheta = currentLocationOdom.theta;
		ninetyRotate = currentLocationOdom.theta;
		
		

		if (step == 1)
		{
			//cout << "First step of driving..." << endl;
			sendDriveCommand(30.0, 30.0);
			Position1X = initialPositionTrackerX;
			Position1Y = initialPositionTrackerY;

			std_msgs::Float32MultiArray initialPop;
			initialPop.layout.dim.push_back(std_msgs::MultiArrayDimension());
			initialPop.layout.dim[0].size = 2;
			initialPop.layout.dim[0].stride = 1;
			initialPop.layout.dim[0].label = "poop";
			//UPDATED FROM 10 TO 25
			initialPop.data.push_back(normalizedValue(currentLocationOdom.x+centerOffsetX));
			initialPop.data.push_back(normalizedValue(currentLocationOdom.y+centerOffsetY));
			//initialPop.data.push_back(roundf((currentLocationOdom.x+centerOffsetX)*25)/25);
			//initialPop.data.push_back(roundf((currentLocationOdom.y+centerOffsetY)*25)/25);
			
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
			if (displacement >= 1.0)
			{
				step = 2;
				Position2X = currentLocationOdom.x;
				Position2Y = currentLocationOdom.y;
				startingTheta = currentLocationOdom.theta;
			}
			cout << "the point: " << initialPop.data[0] << ", " << initialPop.data[1] << " has been inserted/published..." << endl;
			
			//cout << "global coordinate is: " << currentLocationOdom.x + centerOffsetX << ", " << currentLocationOdom.y + centerOffsetY << endl;
		}
		else if (step == 2)
		{
			cout << "step 2: rotating 90 degrees right..." << endl;
			float turnSize = -1.5;
			bool exceedMag = false;

			ninetyRotate = currentLocationOdom.theta;
			if (abs(startingTheta + turnSize) >= 3.142)
			{
				exceedMag = true;
			}
			cout << "exceed magnitude value is " << exceedMag << endl;
			if (exceedMag)
			{
				float desiredTheta = 0.0;

				desiredTheta = 3.142 + (startingTheta - turnSize);
				if (currentLocationOdom.theta <= desiredTheta && currentLocationOdom.theta > 0.0)
				{
					sendDriveCommand(0.0, 0.0);
					cout << "done rotating" << endl;
					step = 3;
				}
				else {
					sendDriveCommand(30.0, -30.0);
					cout << "still rotating to calculated desired theta: " << desiredTheta << endl;
				}
				
				
				
			}
			else
			{
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
			
		}
		else if (step == 3)
		{
			//cout << "Second step of driving..." << endl;
			sendDriveCommand(30.0, 30.0);

			std_msgs::Float32MultiArray initialPopb;
			initialPopb.layout.dim.push_back(std_msgs::MultiArrayDimension());
			initialPopb.layout.dim[0].size = 2;
			initialPopb.layout.dim[0].stride = 1;
			initialPopb.layout.dim[0].label = "poopb";
			//UPDATED FROM 10 TO 25
			initialPopb.data.push_back(normalizedValue(currentLocationOdom.x+centerOffsetX));
			initialPopb.data.push_back(normalizedValue(currentLocationOdom.y+centerOffsetY));
			//initialPopb.data.push_back(roundf((currentLocationOdom.x+centerOffsetX)*25)/25);
			//initialPopb.data.push_back(roundf((currentLocationOdom.y+centerOffsetY)*25)/25);

			visitedLocations[initialPopb.data[0]].insert(initialPopb.data[1]);

			visitedLocationsPublisher.publish(initialPopb);
			cout << "the point: " << initialPopb.data[0] << ", " << initialPopb.data[1] << " has been inserted/published..." << endl;
			
			float displacement = sqrt(((currentLocationOdom.x - Position2X)*(currentLocationOdom.x - Position2X)) + ((currentLocationOdom.y - Position2Y)*(currentLocationOdom.y - Position2Y)));
			if (displacement >= 2.2)
			{
				step = 4;
				Position3X = currentLocationOdom.x;
				Position3Y = currentLocationOdom.y;
				startingTheta = currentLocationOdom.theta;
			}
			//cout << "global coordinate is: " << currentLocationOdom.x + centerOffsetX << ", " << currentLocationOdom.y + centerOffsetY << endl;
			cout << "displacement is: " << displacement << endl;
		}
		else if (step == 4)
		{
			cout << "step 4: rotating 90 degrees right..." << endl;
			float turnSize = -1.5;
			bool exceedMag = false;

			ninetyRotate = currentLocationOdom.theta;
			if (abs(startingTheta + turnSize) >= 3.142)
			{
				exceedMag = true;
			}
			cout << "exceed magnitude value is " << exceedMag << endl;
			if (exceedMag)
			{
				float desiredTheta = 0.0;

				desiredTheta = 3.142 + (startingTheta - turnSize);
				if (currentLocationOdom.theta <= desiredTheta && currentLocationOdom.theta > 0.0)
				{
					sendDriveCommand(0.0, 0.0);
					cout << "done rotating" << endl;
					step = 5;
				}
				else {
					sendDriveCommand(30.0, -30.0);
					cout << "still rotating to calculated desired theta: " << desiredTheta << endl;
				}
				
				
				
			}
			else
			{
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
			
		}
		else if (step == 5)
		{
			//cout << "Third step of driving..." << endl;
			sendDriveCommand(30.0, 30.0);

			std_msgs::Float32MultiArray initialPopc;
			initialPopc.layout.dim.push_back(std_msgs::MultiArrayDimension());
			initialPopc.layout.dim[0].size = 2;
			initialPopc.layout.dim[0].stride = 1;
			initialPopc.layout.dim[0].label = "poopc";
			//UPDATED FROM 10 TO 25
			initialPopc.data.push_back(normalizedValue(currentLocationOdom.x+centerOffsetX));
			initialPopc.data.push_back(normalizedValue(currentLocationOdom.y+centerOffsetY));
			//initialPopc.data.push_back(roundf((currentLocationOdom.x+centerOffsetX)*25)/25);
			//initialPopc.data.push_back(roundf((currentLocationOdom.y+centerOffsetY)*25)/25);

			visitedLocations[initialPopc.data[0]].insert(initialPopc.data[1]);

			visitedLocationsPublisher.publish(initialPopc);
			cout << "the point: " << initialPopc.data[0] << ", " << initialPopc.data[1] << " has been inserted/published..." << endl;
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
			//cout << "current position from Odom is: " << currentLocationOdom.x << ", " << currentLocationOdom.y << endl;
			//cout << "the saved previous position (3X AND 3Y) = " << Position3X << ", " << Position3Y << endl;
			//cout << "third step, displacement is: " << displacement << endl;
			if (displacement >= 2.2)
			{
				step = 6;
				Position4X = currentLocationOdom.x;
				Position4Y = currentLocationOdom.y;
				startingTheta = currentLocationOdom.theta;
			}
		}
		else if (step == 6)
		{
			cout << "step 6: rotating 90 degrees right..." << endl;
			float turnSize = -1.5;
			bool exceedMag = false;

			ninetyRotate = currentLocationOdom.theta;
			if (abs(startingTheta + turnSize) >= 3.142)
			{
				exceedMag = true;
			}
			cout << "exceed magnitude value is " << exceedMag << endl;
			if (exceedMag)
			{
				float desiredTheta = 0.0;

				desiredTheta = 3.142 + (startingTheta - turnSize);
				if (currentLocationOdom.theta <= desiredTheta && currentLocationOdom.theta > 0.0)
				{
					sendDriveCommand(0.0, 0.0);
					cout << "done rotating" << endl;
					step = 7;
				}
				else {
					sendDriveCommand(30.0, -30.0);
					cout << "still rotating to calculated desired theta: " << desiredTheta << endl;
				}
				
				
				
			}
			else
			{
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
		}
		else if (step == 7)
		{
			//cout << "Fourth step of driving..." << endl;
			sendDriveCommand(30.0, 30.0);

			std_msgs::Float32MultiArray initialPopd;
			initialPopd.layout.dim.push_back(std_msgs::MultiArrayDimension());
			initialPopd.layout.dim[0].size = 2;
			initialPopd.layout.dim[0].stride = 1;
			initialPopd.layout.dim[0].label = "poopd";
			//UPDATED FROM 10 TO 25
			initialPopd.data.push_back(normalizedValue(currentLocationOdom.x+centerOffsetX));
			initialPopd.data.push_back(normalizedValue(currentLocationOdom.y+centerOffsetY));
			//initialPopd.data.push_back(roundf((currentLocationOdom.x+centerOffsetX)*25)/25);
		//	initialPopd.data.push_back(roundf((currentLocationOdom.y+centerOffsetY)*25)/25);

			visitedLocations[initialPopd.data[0]].insert(initialPopd.data[1]);

			visitedLocationsPublisher.publish(initialPopd);
			cout << "the point: " << initialPopd.data[0] << ", " << initialPopd.data[1] << " has been inserted/published..." << endl;
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
			if (displacement >= 2.2)
			{
				step = 8;
				Position5X = currentLocationOdom.x;
				Position5Y = currentLocationOdom.y;
				startingTheta = currentLocationOdom.theta;
			}
		}
		else if (step == 8)
		{
			cout << "step 8: rotating 90 degrees right..." << endl;
			float turnSize = -1.5;
			bool exceedMag = false;

			ninetyRotate = currentLocationOdom.theta;
			if (abs(startingTheta + turnSize) >= 3.142)
			{
				exceedMag = true;
			}
			cout << "exceed magnitude value is " << exceedMag << endl;
			if (exceedMag)
			{
				float desiredTheta = 0.0;

				desiredTheta = 3.142 + (startingTheta - turnSize);
				if (currentLocationOdom.theta <= desiredTheta && currentLocationOdom.theta > 0.0)
				{
					sendDriveCommand(0.0, 0.0);
					cout << "done rotating" << endl;
					step = 9;
				}
				else {
					sendDriveCommand(30.0, -30.0);
					cout << "still rotating to calculated desired theta: " << desiredTheta << endl;
				}
				
				
				
			}
			else
			{
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
			
		}
		else if (step == 9)
		{
			//cout << "Fifth and last step of driving..." << endl;
			sendDriveCommand(30.0, 30.0);

			std_msgs::Float32MultiArray initialPope;
			initialPope.layout.dim.push_back(std_msgs::MultiArrayDimension());
			initialPope.layout.dim[0].size = 2;
			initialPope.layout.dim[0].stride = 1;
			initialPope.layout.dim[0].label = "poope";
			//UPDATED FROM 10 TO 25
			initialPope.data.push_back(normalizedValue(currentLocationOdom.x+centerOffsetX));
			initialPope.data.push_back(normalizedValue(currentLocationOdom.y+centerOffsetY));
			//initialPope.data.push_back(roundf((currentLocationOdom.x+centerOffsetX)*25)/25);
			//initialPope.data.push_back(roundf((currentLocationOdom.y+centerOffsetY)*25)/25);

			visitedLocations[initialPope.data[0]].insert(initialPope.data[1]);

			visitedLocationsPublisher.publish(initialPope);
			cout << "the point: " << initialPope.data[0] << ", " << initialPope.data[1] << " has been inserted/published..." << endl;
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
			if (displacement >= 1.4)
			{
				step = 10;
				startingTheta = currentLocationOdom.theta;
				Position6X = currentLocationOdom.x;
				Position6Y = currentLocationOdom.y;

			}
		}
		else if (step == 10)
		{	
			cout << "step 2: rotating 90 degrees left..." << endl;
			float turnSize = 1.5;
			bool exceedMag = false;

			ninetyRotate = currentLocationOdom.theta;
			if (abs(startingTheta + turnSize) >= 3.142)
			{
				exceedMag = true;
			}
			cout << "exceed magnitude value is " << exceedMag << endl;
			if (exceedMag)
			{
				float desiredTheta = 0.0;

				desiredTheta = -3.142 + (startingTheta - turnSize);
				if (currentLocationOdom.theta >= desiredTheta && currentLocationOdom.theta < 0.0)
				{
					sendDriveCommand(0.0, 0.0);
					cout << "done rotating" << endl;
					step = 11;
					//initialMove = false;
				}
				else {
					sendDriveCommand(-30.0, 30.0);
					cout << "still rotating to calculated desired theta: " << desiredTheta << endl;
				}
				
				
				
			}
			else
			{
			      if (abs(ninetyRotate - startingTheta) >= 1.5)
			      {
				    sendDriveCommand(0.0, 0.0); 
				     cout << "done rotating" << endl;
				    step = 11;
				      //initialMove = false;

			      }
			      else {
				    sendDriveCommand(-30.0, 30.0);
			      }
			}
			
		}
		else if (step == 11)
		{
			//NEW BLOK FOR INITAL POP IDEA
			
			
			cout << "Moving into place to begin spiral search..." << endl;
			sendDriveCommand(30.0, 30.0);

			std_msgs::Float32MultiArray initialPopf;
			initialPopf.layout.dim.push_back(std_msgs::MultiArrayDimension());
			initialPopf.layout.dim[0].size = 2;
			initialPopf.layout.dim[0].stride = 1;
			initialPopf.layout.dim[0].label = "poopf";
			//UPDATED FROM 10 TO 25
			initialPopf.data.push_back(normalizedValue(currentLocationOdom.x+centerOffsetX));
			initialPopf.data.push_back(normalizedValue(currentLocationOdom.y+centerOffsetY));
			//initialPopf.data.push_back(roundf((currentLocationOdom.x+centerOffsetX)*25)/25);
			//initialPopf.data.push_back(roundf((currentLocationOdom.y+centerOffsetY)*25)/25);

			visitedLocations[initialPopf.data[0]].insert(initialPopf.data[1]);

			visitedLocationsPublisher.publish(initialPopf);
			cout << "the point: " << initialPopf.data[0] << ", " << initialPopf.data[1] << " has been inserted/published..." << endl;
			
			float displacement = sqrt(((currentLocationOdom.x - Position6X)*(currentLocationOdom.x - Position6X)) + ((currentLocationOdom.y - Position6Y)*(currentLocationOdom.y - Position6Y)));
			if (displacement >= 0.65)
			{
				step = 12;
				startingTheta = currentLocationOdom.theta;

			}
		}
		else if (step == 12)
		{
			cout << "rotating right to begin spiral search..." << endl;
			float turnSize = -1.5;
			bool exceedMag = false;

			ninetyRotate = currentLocationOdom.theta;
			if (abs(startingTheta + turnSize) >= 3.142)
			{
				exceedMag = true;
			}
			cout << "exceed magnitude value is " << exceedMag << endl;
			if (exceedMag)
			{
				float desiredTheta = 0.0;

				desiredTheta = 3.142 + (startingTheta - turnSize);
				if (currentLocationOdom.theta <= desiredTheta && currentLocationOdom.theta > 0.0)
				{
					sendDriveCommand(0.0, 0.0);
					cout << "done rotating" << endl;
					step = 13;
					initialMove = false;
					mapTesting = true;
				}
				else {
					sendDriveCommand(30.0, -30.0);
					cout << "still rotating to calculated desired theta: " << desiredTheta << endl;
				}
				
				
				
			}
			else
			{
			      if (abs(ninetyRotate - startingTheta) >= 1.5)
			      {
				    sendDriveCommand(0.0, 0.0); 
				     cout << "done rotating" << endl;
				    step = 13;
				      initialMove = false;
				      mapTesting = true;

			      }
			      else {
				    sendDriveCommand(30.0, -30.0);
			      }
			}
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
		
		myCoordinate.data.push_back(normalizedValue(currentLocationOdom.x+centerOffsetX));
		myCoordinate.data.push_back(normalizedValue(currentLocationOdom.y+centerOffsetY));
		
		visitedLocations[myCoordinate.data[0]].insert(myCoordinate.data[1]);

		visitedLocationsPublisher.publish(myCoordinate);
	
		/*if (startingTheta <= -3.1)
		{
			startingTheta = 3.12;
		}
		if (ninetyRotate <= -3.1)
		{
			ninetyRotate = 3.12;
		}
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
	      }*/
		cout << "step 2: rotating 90 degrees left..." << endl;
		
		geometry_msgs::Point tempLocal;
		
			float turnSize = 1.5;
			bool exceedMag = false;

			ninetyRotate = currentLocationOdom.theta;
			//if the ninety degree turn crosses the pi line, this is a special condition
			if (abs(startingTheta + turnSize) >= 3.14159)
			{
				exceedMag = true;
			}
			cout << "exceed magnitude value is " << exceedMag << endl;
			if (exceedMag)
			{
				float desiredTheta = 0.0;

				desiredTheta = -3.142 + (startingTheta - turnSize);
				if (currentLocationOdom.theta >= desiredTheta && currentLocationOdom.theta < 0.0)
				{
					sendDriveCommand(0.0, 0.0);
					rotateBool = false;
					hardcodedPop = true;
				    	 //initialMove = true;
				      //step = 1;
					initialPositionTrackerX = currentLocationOdom.x;
					initialPositionTrackerY = currentLocationOdom.y;
					
				     cout << "done rotating" << endl;
				}
				else {
					sendDriveCommand(-30.0, 30.0);
					cout << "still rotating to calculated desired theta: " << desiredTheta << endl;
				}
				
				
				
			}
			else
			{	//ninetyRotate = current theta
				//if the diff between the current theta and starting theta is >= 90 degrees, stop
			      if (abs(ninetyRotate - startingTheta) >= 1.5)
			      {
				   sendDriveCommand(0.0, 0.0);
					rotateBool = false;
				      	hardcodedPop = true;
				    	 //initialMove = true;
				      //GPSCenter = true;
				      //step = 1;
					initialPositionTrackerX = currentLocationOdom.x;
					initialPositionTrackerY = currentLocationOdom.y;
				     cout << "done rotating" << endl;

			      }	//else, turn right
			      else {
				    sendDriveCommand(-30.0, 30.0);
			      }
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
			geometry_msgs::Point temp;
			temp.x = currentLocationMap.x;
			temp.y = currentLocationMap.y;
			  //SET the centerMap location by passing that variable here		

		      //startTime = getROSTimeInMilliSecs();
			//TEMPORARY TESTING FOR CENTER OFFSET
			//2/11/19, increased intensity of starting positions, from 0.2 to 0.325 to 0.5 and now 0.45
			//calculates the center offset based on starting location
			if ((currentLocationOdom.theta >= -0.15 && currentLocationOdom.theta <= 0) || (currentLocationOdom.theta<= 0.15 && currentLocationOdom.theta >= 0))
			{
				centerLocationOdom.x = currentLocationOdom.x - 1.0 - 0.45;
				centerLocationOdom.y = currentLocationOdom.y;
				centerOffsetX = -1.0 -0.45;
				centerOffsetY = 0.0;
				cout << "current location is: " << currentLocationOdom.x + 1.45 << ", " << currentLocationOdom.y << endl;
			}
			else if(currentLocationOdom.theta >= 0.6 && currentLocationOdom.theta<= 0.9)
			{
				centerLocationOdom.x = currentLocationOdom.x - 1.207*cos(3.142/4) - 0.45;
				centerLocationOdom.y = currentLocationOdom.y - 1.207*sin(3.142/4) - 0.45;
				centerOffsetX = -1.207*cos(3.142/4) - 0.45;
				centerOffsetY = -1.207*sin(3.142/4) - 0.45;
				cout << "current location is: " << currentLocationOdom.x + 1.207*cos(3.142/4) + 0.45<< ", " << currentLocationOdom.y + 1.207*sin(3.142/4) +0.5<< endl;
			}
			else if(currentLocationOdom.theta >= 1.35 && currentLocationOdom.theta<= 1.65)
			{
				centerLocationOdom.x = currentLocationOdom.x;
				centerLocationOdom.y = currentLocationOdom.y - 1.0 - 0.45;
				centerOffsetX = 0.0;
				centerOffsetY = -1.0 - 0.45;
				cout << "current location is: " << currentLocationOdom.x << ", " << currentLocationOdom.y + 0.45 + 1.0 << endl;
			}
			else if(currentLocationOdom.theta >= 2.1 && currentLocationOdom.theta<= 2.4)
			{
				centerLocationOdom.x = currentLocationOdom.x + 1.207*cos(3.142/4) + 0.45;
				centerLocationOdom.y = currentLocationOdom.y - 1.207*sin(3.142/4) - 0.45;
				centerOffsetX = 1.207*cos(3.142/4) + 0.45;
				centerOffsetY = -1.207*sin(3.142/4) - 0.45;
				cout << "current location is: " << currentLocationOdom.x - 1.207*cos(3.142/4) - 0.45 << ", " << currentLocationOdom.y + 1.207*sin(3.142/4) + 0.45<< endl;
			}
			else if((currentLocationOdom.theta >= 2.85) || (currentLocationOdom.theta <= -2.85))
			{
				centerLocationOdom.x = currentLocationOdom.x + 1.0 + 0.45;
				centerLocationOdom.y = currentLocationOdom.y;
				centerOffsetX = 1.0 + 0.45;
				centerOffsetY = 0.0;
				cout << "current location is: " << currentLocationOdom.x - 1.0 - 0.45 << ", " << currentLocationOdom.y << endl;
			}
			else if(currentLocationOdom.theta >= -2.4 && currentLocationOdom.theta <= -2.1)
			{
				centerLocationOdom.x = currentLocationOdom.x + 1.207*cos(3.142/4) + 0.45;
				centerLocationOdom.y = currentLocationOdom.y + 1.207*sin(3.142/4) + 0.45;
				centerOffsetX = 1.207*cos(3.142/4) + 0.45;
				centerOffsetY = 1.207*sin(3.142/4) + 0.45;
				cout << "current location is: " << currentLocationOdom.x - 1.207*cos(3.142/4) - 0.45 << ", " << currentLocationOdom.y - 1.207*sin(3.142/4) - 0.45 << endl;
			}
			else if(currentLocationOdom.theta >= -1.65 && currentLocationOdom.theta <= -1.35)
			{
				centerLocationOdom.x = currentLocationOdom.x;
				centerLocationOdom.y = currentLocationOdom.y + 1.0 + 0.45;
				centerOffsetX = 0.0;
				centerOffsetY = 1.0 + 0.45;
				cout << "current location is: " << currentLocationOdom.x << ", " << currentLocationOdom.y - 1.0 - 0.45<< endl;
			}
			else if(currentLocationOdom.theta >= -0.9 && currentLocationOdom.theta<= -0.6)
			{
				centerLocationOdom.x = currentLocationOdom.x - 1.207*cos(3.142/4) - 0.45;
				centerLocationOdom.y = currentLocationOdom.y + 1.207*sin(3.142/4) + 0.45;
				centerOffsetX = -1.207*cos(3.142/4) - 0.45;
				centerOffsetY = 1.207*sin(3.142/4) + 0.45;
				cout << "current location is: " << currentLocationOdom.x + 1.207*cos(3.142/4) + 0.45 << ", " << currentLocationOdom.y - 1.207*sin(3.142/4) - 0.45 << endl;
			}
			
			rotateBool = true;
		}
    		else
    		{
      			return;
    		}
  	}
	  
	humanTime();
	
		
}

void spiralSearch(const ros::TimerEvent&)
{
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
		myCoordinate.layout.dim[0].label = "fart";

		//cout << "float32multiarray has been instantiated...\n";
		//UPDATED FROM 10 TO 25
		myCoordinate.data.push_back(normalizedValue(currentLocationOdom.x+centerOffsetX));
		myCoordinate.data.push_back(normalizedValue(currentLocationOdom.y+centerOffsetY));
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
		std_msgs::Float32MultiArray frontCheckCoord;
		
		frontCheckCoord.layout.dim.push_back(std_msgs::MultiArrayDimension());
		frontCheckCoord.layout.dim[0].size = 2;
		frontCheckCoord.layout.dim[0].stride = 1;
		frontCheckCoord.layout.dim[0].label = "front";
		
		checkCoord.layout.dim.push_back(std_msgs::MultiArrayDimension());
		checkCoord.layout.dim[0].size = 2;
		checkCoord.layout.dim[0].stride = 1;
		checkCoord.layout.dim[0].label = "check";
		//float hypot = sqrt(((currentLocationOdom.x + centerOffsetX) * (currentLocationOdom.x + centerOffsetX))+((currentLocationOdom.y + centerOffsetY) * (currentLocationOdom.y + centerOffsetY)));
		//SUBTRACT a small constant from hypotenuse
		//hypot = hypot - 0.2;
		//NEW METHOD, vector addition
		float newTheta = currentLocationOdom.theta - 1.53;
		checkCoord.data.push_back(normalizedValue(centerOffsetX + currentLocationOdom.x + 0.25*cos(newTheta)));
		checkCoord.data.push_back(normalizedValue(centerOffsetY + currentLocationOdom.y + 0.25*sin(newTheta)));
		
		float leftDrive = 0.0;
		float rightDrive = 0.0;

		//NEWEST METHOD CALCULATING ERROR
		bool maxFrontError = false;
		float FrontError;
		int m = 0;
		float newX;
		float newY;
		//starts at 2 so that it doesnt check on top of itself; checks 2 meters in front of it
		for (m = 2; m <= 8; m++)
		{
			newX = normalizedValue(currentLocationOdom.x + centerOffsetX + m*0.25*cos(currentLocationOdom.theta));
			newY = normalizedValue(currentLocationOdom.y + centerOffsetY + m*0.25*sin(currentLocationOdom.theta));
			cout << "current x, y" << currentLocationOdom.x + centerOffsetX << ", " << currentLocationOdom.y + centerOffsetY << endl;
			if (isVisited(normalizedValue(newX), normalizedValue(newY)))
			{
				cout << "Front vector ended at m = " << m << "after checking: " << newX << ", " << newY << endl;
				maxFrontError = false;
				break;
			}
			else if (m == 8){
				maxFrontError = true;
			}
		}
		if (!maxFrontError)
		{
			float xWall = normalizedValue(newX);
			float yWall = normalizedValue(newY);
			FrontError = sqrt((xWall - (centerOffsetX+currentLocationOdom.x))*(xWall - (centerOffsetX+currentLocationOdom.x)) + (yWall - (centerOffsetY+currentLocationOdom.y))*(yWall - (centerOffsetY+currentLocationOdom.y)));
		}
		else {
			FrontError = 2;
		}
		cout << "FrontError is: " << FrontError << endl;
		if (FrontError <= 0.25)
		{
			leftDrive = -100;
			rightDrive = 100;
		}
		else if (FrontError <= 0.5)
		{
			leftDrive = FrontError*100 - 125;
			rightDrive = 100;
		}
		else {
			leftDrive = 100*FrontError - 100;
			rightDrive = 100;
		}

	
		bool maxRightError = false;
		float RightError;
		int n = 0;
		for (n = 1; n <= 4; n++)
		{
			newX = normalizedValue(currentLocationOdom.x + centerOffsetX + n*0.25*cos(currentLocationOdom.theta - 1.571));
			newY = normalizedValue(currentLocationOdom.y + centerOffsetY + n*0.25*sin(currentLocationOdom.theta - 1.571));
			
			if (isVisited(normalizedValue(newX), normalizedValue(newY)))
			{
				cout << "Right vector ended at n = " << n << "after checking: " << normalizedValue(newX) << ", " << normalizedValue(newY) << endl;
				maxRightError = false;
				break;
			}
			else if (n == 4) {
				maxRightError = true;
				cout << "Right vector max distance after checking: " << normalizedValue(newX) << ", " << normalizedValue(newY) << endl;
			}
		}
		if (!maxRightError)
		{
			float xWall = normalizedValue(newX);
			float yWall = normalizedValue(newY);
			cout << "RightError calculated with currentLocation and xWall,yWall = " << xWall << ", " << yWall << endl;
			RightError = sqrt((xWall - (centerOffsetX+currentLocationOdom.x))*(xWall - (centerOffsetX+currentLocationOdom.x)) + (yWall - (centerOffsetY+currentLocationOdom.y))*(yWall - (centerOffsetY+currentLocationOdom.y)));
		}
		else {
			cout << "max RightError" << endl;
			RightError = 1;
		}
		//RightError = RightError - 0.25;
		cout << "RightError is: " << RightError << ", desired is 0.25" << endl;
		cout << "CurrentLocation X,Y: " << currentLocationOdom.x + centerOffsetX << ", " << currentLocationOdom.y + centerOffsetY << endl;
		RightError = RightError - 0.25;
		if (maxFrontError)
		{
			if (RightError < 0)
			{
				leftDrive = 100 + RightError*200;
				//leftDrive = leftDrive + RightError*200;
			}
			//THIS ENTIRE ELSE IF BLOCK IS (barely)TESTED
			else if (RightError <= 0.25) {
				//rightDrive = 100 - (100*RightError);
				//rightDrive = -50;
				leftDrive = (200*RightError)+50;
				rightDrive = 100;
			}
			else {
				rightDrive = 100 - (200*RightError);
				//rightDrive = rightDrive - (200*RightError);
			}
		}
		cout << "sending drive commands: " << leftDrive << ", " << rightDrive << endl;
		sendDriveCommand(leftDrive, rightDrive);
		//CALCULATE new x,y
		//UPDATED FROM 10 TO 25
		//checkCoord.data.push_back(roundf((hypot*cos(currentLocationOdom.theta))*25)/25);
		//checkCoord.data.push_back(normalizedValue(hypot*cos(currentLocationOdom.theta)));
		//checkCoord.data.push_back(normalizedValue(hypot*sin(currentLocationOdom.theta)));
		//PREVIOUSLY 0.25 r, now 0.5
		frontCheckCoord.data.push_back(normalizedValue((currentLocationOdom.x + centerOffsetX + 0.5*cos(currentLocationOdom.theta))));
		frontCheckCoord.data.push_back(normalizedValue((currentLocationOdom.y + centerOffsetY + 0.5*sin(currentLocationOdom.theta))));
		
		//cout << "checking location in FRONT: " << normalizedValue((currentLocationOdom.x + centerOffsetX + 0.5*cos(currentLocationOdom.theta))) << ", " << normalizedValue((currentLocationOdom.y + centerOffsetY + 0.5*sin(currentLocationOdom.theta))) << endl;
		//cout << "checking location on RIGHT: " << normalizedValue(centerOffsetX + currentLocationOdom.x + 0.25*cos(newTheta)) << ", " << normalizedValue(centerOffsetY + currentLocationOdom.y + 0.25*sin(newTheta)) << endl;
		//frontCheckCoord.data.push_back(roundf((currentLocationOdom.x + centerOffsetX + 0.2*cos(currentLocationOdom.theta))*10)/10);
		if (visitedLocations.find(frontCheckCoord.data[0]) != visitedLocations.end())
		{
			if (visitedLocations[frontCheckCoord.data[0]].find(frontCheckCoord.data[1]) != visitedLocations[frontCheckCoord.data[0]].end())
			{
				//cout << "location in front: " << frontCheckCoord.data[0] << ", " << frontCheckCoord.data[1] << " HAS been visited" << endl;
				//sendDriveCommand(-50.0, 50.0);
			}
			else
			{
				if (visitedLocations.find(checkCoord.data[0]) == visitedLocations.end())
				{	//right is unvisited
					//cout << "Location on right: " << checkCoord.data[0] << ", " << checkCoord.data[1] << " has NOT been visited" << endl;
					//sendDriveCommand(100.0, -50.0);
				}
				else
				{
					if (visitedLocations[checkCoord.data[0]].find(checkCoord.data[1]) == visitedLocations[checkCoord.data[0]].end())
					{
						//cout << "Location on right: " << checkCoord.data[0] << ", " << checkCoord.data[1] << " has NOT been visited" << endl;
						//sendDriveCommand(100.0,-50.0);
					}
					else
					{
						//sendDriveCommand(100.0, 100.0);
					}
					
				}
			}
		}
		else {
			if (visitedLocations.find(checkCoord.data[0]) == visitedLocations.end())
			{	//right is unvisited
				//cout << "Location on right: " << checkCoord.data[0] << ", " << checkCoord.data[1] << " has NOT been visited" << endl;
				//sendDriveCommand(100.0, -50.0);
			}
			else
			{
				if (visitedLocations[checkCoord.data[0]].find(checkCoord.data[1]) == visitedLocations[checkCoord.data[0]].end())
				{
					//cout << "Location on right: " << checkCoord.data[0] << ", " << checkCoord.data[1] << " has NOT been visited" << endl;
					//sendDriveCommand(100.0, -50.0);
				}
				else
				{
					//sendDriveCommand(100.0, 100.0);
				}
				
			}
			
		}
		
	}
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
	    //vector<Tag> tags;
	
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
		if (mapTesting)
		{
			aprilTagDetected = true;
			tagIndex++;
		}
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
void nameHandler(const geometry_msgs::Point::ConstPtr& msg)
{
	cout << "nameHandeler called, starting location GPS is: " << endl;
	cout << "x: " << msg->x << ", y: " << msg->y << endl;

	startingLocationGPS.push_back(*msg);

	// print current size of vector
	for (int i=0; i< startingLocationGPS.size(); i++)     
        cout << startingLocationGPS[i].x << ", " << startingLocationGPS[i].y << endl;
        cout << "Size of vector is: " << startingLocationGPS.size() << endl;
	
}

void visitedLocationsHandler(const std_msgs::Float32MultiArray::ConstPtr& msg) {
	
	//std_msgs::Float32MultiArray receivedCoordinate[2];
	float x = msg->data[0];
	float y = msg->data[1];
	visitedLocations[x].insert(y);

	//check if swarmieid is set
	if (!swarmieidset && swarmieid != 0){
		markerid = swarmieid * 10000000;
		swarmieidset = true;
	} 

	visualization_msgs::Marker marker;
   	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
   	marker.header.frame_id = publishedName + "/map";
   	marker.header.stamp = ros::Time::now();

    	// Set the namespace and id for this marker.  This serves to create a unique ID
    	// Any marker sent with the same namespace and id will overwrite the old one
   	marker.ns = "basic_shapes";
   	marker.id = markerid;
    	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    	marker.type = shape;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	    marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.25;
	marker.scale.y = 0.25;
	marker.scale.z = 0.25;
	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();

	marker_pub.publish(marker);
	markerid = markerid + 1;

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
  robotnamePublisher.publish(msg);
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

float normalizedValue(float x)
{
	float temp = x * 100;
	int val = round(temp);
	
	if ((val % 25) > 13)
	{
		int n = (val/25) + 1;
		return n*0.25;
	}
	else
	{
		int n = (val/25);
		return n*0.25;
	}
}

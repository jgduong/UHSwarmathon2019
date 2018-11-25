#include <ros/ros.h>
#include <string>

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
#include "swarmie_msgs/Waypoint.h"
//#include "swarmie_msgs/InfoMessage.h"

// Include Controllers
#include <vector>

#include "Point.h"
#include "Tag.h"

/****************
 * New Includes *
 8****************/
/*include "state_machine/StateMachine.h"
#include "waypoints/SimpleWaypoint.h"
#include "logic/LogicMachine.h"
#include "logic/SearchState.h"
#include "logic/PickUpState.h"
#include "logic/FindHomeState.h"
#include "logic/AvoidState.h"
#include "logic/AvoidHomeState.h"
#include "logic/AvoidCubeState.h"
#include "logic/DropOffState.h"
#include "logic/InitState.h"
#include "Gripper.h"
#include "MotorController.h"
#include "TagUtilities.h"
*/
// To handle shutdown signals so the node quits
// properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

#include <exception> // For exception handling

using namespace std;


/************************
 * Global Alphabet Soup *
 ************************/

// Random number generator
random_numbers::RandomNumberGenerator* rng;

// Create logic controller

void humanTime();

// Behaviours Logic Functions
void sendDriveCommand(double linearVel, double angularVel);
//void sendGripperPosition( Gripper::Position pos );

int currentMode = 0;
const float state_machines_loop = 0.1; // time between state machines function call
const float status_publish_interval = 1;
const float heartbeat_publish_interval = 2;
const float info_publish_interval = 1;
const float waypoint_tolerance = 0.1; //10 cm tolerance.
const float state_interval = 0.1;

bool publish_info = false;

float prevWrist = 0;
float prevFinger = 0;
long int startTime = 0;
float minutesTime = 0;
float hoursTime = 0;

float drift_tolerance = 0.5; // meters

std_msgs::String msg;


string roverName = "";

char host[128];
char prev_state_machine[128];
// records time for delays in sequanced actions, 1 second resolution.
time_t timerStartTime;

// An initial delay to allow the rover to gather enough position data to 
// average its location.
unsigned int startDelayInSeconds = 30;
float timerTimeElapsed = 0;

// Converts the time passed as reported by ROS (which takes Gazebo simulation rate into account) into milliseconds as an integer.
long int getROSTimeInMilliSecs();

/*enum states {
	waiting = 0,
	forward,
	right
	
};
enum states current_state = waiting;*/

int current_state = 0;

/****************************
 * END ALPHABET GLOBAL SOUP *
 ****************************/


/******************
 * ROS Publishers *
 ******************/
ros::Publisher state_machine_publish;
ros::Publisher status_publisher;
ros::Publisher finger_angle_publish;
ros::Publisher wrist_angle_publish;
ros::Publisher info_log_publisher;
ros::Publisher drive_control_publish;
ros::Publisher heartbeat_publisher;
//ros::Publisher rover_info_publisher;
//ros::Publisher rover_info_timer_publisher;

void setupPublishers( ros::NodeHandle &ros_handle, string published_name )
{
    /*status_publisher = ros_handle.advertise<std_msgs::String>((published_name + "/status"), 1, true);
    state_machine_publish = ros_handle.advertise<std_msgs::String>((published_name + "/state_machine"), 1, true);
    finger_angle_publish = ros_handle.advertise<std_msgs::Float32>((published_name + "/fingerAngle/cmd"), 1, true);
    wrist_angle_publish = ros_handle.advertise<std_msgs::Float32>((published_name + "/wristAngle/cmd"), 1, true);
    */
    info_log_publisher = ros_handle.advertise<std_msgs::String>("/infoLog", 1, true);
    drive_control_publish = ros_handle.advertise<geometry_msgs::Twist>((published_name + "/driveControl"), 10);
    heartbeat_publisher = ros_handle.advertise<std_msgs::String>((published_name + "/behaviour/heartbeat"), 1, true);
	
    //rover_info_timer_publisher = ros_handle.advertise<std_msgs::String>((published_name + "/infoTimer"), 1, true);
    //rover_info_publisher = ros_handle.advertise<swarmie_msgs::InfoMessage>(("roverInfo"), 6, true);

    roverName = published_name;
}

/*******************
 * ROS Subscribers *
 *******************/
ros::Subscriber joy_subscriber;
ros::Subscriber mode_subscriber;
ros::Subscriber target_subscriber;
ros::Subscriber raw_odom_subscriber;
ros::Subscriber odometry_subscriber;
ros::Subscriber map_subscriber;
ros::Subscriber rover_info_subscriber;
ros::Subscriber virtualFenceSubscriber; //receives data for vitrual boundaries

/******************************************
 * ROS Callback Functions for Subscribers *
 ******************************************/
void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);
void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tagInfo);
void odomHandler(const nav_msgs::Odometry::ConstPtr& message);
void odomAndAccelHandler(const nav_msgs::Odometry::ConstPtr& message);
void odomAccelAndGPSHandler(const nav_msgs::Odometry::ConstPtr& message);
void manualWaypointHandler(const swarmie_msgs::Waypoint& message);
void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight);
void virtualFenceHandler(const std_msgs::Float32MultiArray& message); //Used to set an invisible boundary for robots to keep them from traveling outside specific bounds
//void roverInfoHandler(const swarmie_msgs::InfoMessage& message);

void setupSubscribers( ros::NodeHandle &ros_handle, string published_name )
{
    /*joy_subscriber = ros_handle.subscribe((published_name + "/joystick"), 10, joyCmdHandler);
    mode_subscriber = ros_handle.subscribe((published_name + "/mode"), 1, modeHandler);
    target_subscriber = ros_handle.subscribe((published_name + "/targets"), 10, targetHandler);
    raw_odom_subscriber = ros_handle.subscribe((published_name + "/odom/"), 10, odomHandler);
    odometry_subscriber = ros_handle.subscribe((published_name + "/odom/filtered"), 10, odomAndAccelHandler);
    map_subscriber = ros_handle.subscribe((published_name + "/odom/ekf"), 10, odomAccelAndGPSHandler);
    */
    //rover_info_subscriber = ros_handle.subscribe("/roverInfo", 10, roverInfoHandler);
    virtualFenceSubscriber = mNH.subscribe(("/virtualFence"), 10, virtualFenceHandler); //receives data for vitrual boundaries

}

/**************
 * ROS Timers *
 **************/
ros::Timer state_machine_timer;
ros::Timer publish_status_timer;
ros::Timer publish_heartbeat_timer;
ros::Timer publish_info_timer;



/***********************
 * ROS Timer Functions *
 ***********************/
void runStateMachines(const ros::TimerEvent&);
void publishStatusTimerEventHandler(const ros::TimerEvent& event);
void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);
//void publishRoverInfoTimerEventHandler(const ros::TimerEvent& event);

void check_the_state(const ros::TimerEvent&);


void toggle_movement(const ros::TimerEvent&);


void setupTimerCallbacks( ros::NodeHandle &ros_handle )
{
    /*publish_status_timer = ros_handle.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    state_machine_timer = ros_handle.createTimer(ros::Duration(state_machines_loop), runStateMachines);
    */
    publish_heartbeat_timer = ros_handle.createTimer(ros::Duration(heartbeat_publish_interval), publishHeartBeatTimerEventHandler);
    
    //publish_info_timer = ros_handle.createTimer(ros::Duration(info_publish_interval), publishRoverInfoTimerEventHandler);
}

/******************
 * SIGINT Handler *
 ******************/
void sigintEventHandler(int signal);

/*****************
 * Sensor Inputs *
 *****************/
//LogicInputs inputs;
//LogicOutputs outputs;
//IOTable iotable = { &inputs, &outputs };
//iotable.inputs = &inputs;
//iotable.outputs = &outputs;
/***********************
 * Logic State Machine *
 ***********************/
//LogicMachine logic_machine( &iotable );
 //   InitState init_state( &iotable );


void setupLogicMachine()
{
    /* add States */
   // logic_machine.addState( init_state.getIdentifier(), dynamic_cast<State *>(&init_state) );

    return;
}





/*****************
 * MAIN FUNCTION *
 *****************/

int main(int argc, char **argv)
{
    gethostname(host, sizeof (host));
    string hostname(host);
    string published_name;

    //Determine this Rover's name
    if (argc >= 2)
    {
        published_name = argv[1];
        cout << "Welcome to the world of tomorrow " << published_name
             << "!  Behaviour turnDirectionule started." << endl;
    }
    else
    {
        published_name = hostname;
        cout << "No Name Selected. Default is: " << published_name << endl;
    }

    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (published_name + "_BEHAVIOUR"), ros::init_options::NoSigintHandler);
    ros::NodeHandle ros_handle;

ros::Timer check_state_timer;

ros::Timer state_switch_timer;

check_state_timer = ros_handle.createTimer(ros::Duration(state_interval), check_the_state);

//state_switch_timer = ros_handle.createTimer(ros::Duration(2), toggle_movement);

    // Register the SIGINT event handler so the node can shutdown properly
    //signal(SIGINT, sigintEventHandler);

    setupSubscribers( ros_handle, published_name );

    //Sonar Stuff
    message_filters::Subscriber<sensor_msgs::Range> sonar_left_subscriber(ros_handle, (published_name + "/sonarLeft"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonar_center_subscriber(ros_handle, (published_name + "/sonarCenter"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonar_right_subscriber(ros_handle, (published_name + "/sonarRight"), 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;
    message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonar_left_subscriber, sonar_center_subscriber, sonar_right_subscriber);
    sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));

    
    setupPublishers( ros_handle, published_name );
    setupTimerCallbacks( ros_handle );
    //setupLogicMachine();

    //TBD How to wrap this section up
    //std_msgs::String msg;
    //msg.data = "Log Started";
    //info_log_publisher.publish(msg);

    stringstream ss;
    ss << "Rover start delay set to " << startDelayInSeconds << " seconds";
    msg.data = ss.str();
    //info_log_publisher.publish(msg);
    timerStartTime = time(0);


    //swarmie_msgs::InfoMessage infoMsg;
    //infoMsg.name = roverName;
    /*
    infoMsg.x = inputs.odom_accel_gps.x;
    infoMsg.y = inputs.odom_accel_gps.y;
    infoMsg.sonar_left = inputs.us_left;
    infoMsg.sonar_right = inputs.us_right;
    infoMsg.sonar_center = inputs.us_center;
    infoMsg.state = logic_machine.getCurrentIdentifier();
    infoMsg.number_of_cubes = TagUtilities::numberOfTags(&inputs.tags, 0);
    infoMsg.number_of_base_tags = TagUtilities::numberOfTags(&inputs.tags, 256);
    */
    //rover_info_publisher.publish(infoMsg);

    //inputs.rover_name = published_name;

    ros::spin();

    return EXIT_SUCCESS;
}








/************************
 * Subsequent Functions *
 ************************/

// This is the top-most logic control block organised as a state machine.
// This function calls the dropOff, pickUp, and search controllers.
// This block passes the goal location to the proportional-integral-derivative
// controllers in the abridge package.

void check_the_state(const ros::TimerEvent&)
{
	//check to see if this periodic function is running
	std_msgs::String msg;
 	msg.data = "Checking State...";
  	info_log_publisher.publish(msg);
	
	if (current_state == 0)
	{
		std::cout << "waiting\n";
	}
	if (current_state == 1)
	{	//straight
		geometry_msgs::Twist vel;
		vel.linear.x = 200;
		//vel.angular.z = 200;
		
		drive_control_publish.publish(vel);
	}
	if (current_state == 2)
	{
		//turn right
		geometry_msgs::Twist vel;
		vel.linear.x = -200;
		vel.angular.z = 200;
		
		drive_control_publish.publish(vel);
	}
}
/*
void toggle_movement(const ros::TimerEvent&)
{
	if (current_state == 1)
	{
		current_state = 1;
	}
	else {
		current_state = 1;
	}
}
*/
void publishHeartBeatTimerEventHandler(const ros::TimerEvent&) {
  std_msgs::String msg;
  msg.data = "";
  heartbeat_publisher.publish(msg);
}

void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight) {
  std_msgs::String msg;
	/*
  msg.data = "sonarLeft: ";
  info_log_publisher.publish(msg);
  	msg.data = to_string(sonarLeft->range);
  info_log_publisher.publish(msg);
	
  msg.data = "sonarRight: ";
  info_log_publisher.publish(msg);
	msg.data = to_string(sonarRight->range);
  info_log_publisher.publish(msg);
*/	
   msg.data = "sonarCenter: ";
  info_log_publisher.publish(msg);
  msg.data = to_string(sonarCenter->range);
  info_log_publisher.publish(msg);
	
  if (sonarCenter->range <= 0.5 || sonarLeft->range <= 0.5 || sonarRight->range <= 0.5) {
	current_state = 2;  
  }
  else {
	  current_state = 1;
  }
  
  //logicController.SetSonarData(sonarLeft->range, sonarCenter->range, sonarRight->range);
  
}


// Allows a virtual fence to be defined and enabled or disabled through ROS
void virtualFenceHandler(const std_msgs::Float32MultiArray& message) 
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
}







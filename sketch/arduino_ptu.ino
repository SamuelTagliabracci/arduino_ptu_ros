#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <Servo.h>

//Now you can send all of the statements you want
//to be logged at the appropriate verbosity level

//  nh.logdebug("Debug Statement");
//  nh.loginfo("Program info");
//  nh.logwarn("Warnings.");
//  nh.logerror("Errors..");
//  nh.logfatal("Fatalities!");

const float Pi = 3.14159;

//Setup Servo Objects
Servo ptu_pan;
Servo ptu_tilt;

//Main Ros Node Handler
ros::NodeHandle nh;

//Setup Current, Minimum, and Maximum Variables in Radians for Pan & Tilt
float pan_angle = 1.59;
float pan_min = 0;
float pan_max = 3.14;

float tilt_angle = 1.59;
float tilt_min = 0;
float tilt_max = 2.25;

char *joint_name[] = {"ptu_pan_joint", "ptu_tilt_joint"};
float joint_position[]= {0, 0};
float joint_velocity[]= {0.5, 0.5};
float joint_effort[]= {1000, 1000};

geometry_msgs::Twist ptu_cmd_vel;

sensor_msgs::JointState ptu_jointstatus;

//Publish JointState msgs
ros::Publisher pub_ptu("/ptu/joint_states", &ptu_jointstatus);

//Create a Callback that is Triggered when /ptu/cmd_vel/ Twist msgs come in
void ptu_cb( const geometry_msgs::Twist &msg)
{
  //nh.logerror("Twist Recieved");
  
  pan_angle = ((msg.angular.z * -1 + 1) / 2) * Pi;
  tilt_angle = ((msg.angular.y * -1 + 1) / 2) * Pi;

  joint_position[0] = pan_angle;
  joint_position[1] = tilt_angle;
}

//Subscribe to /ptu/cmd_vel topic for Twist msgs
ros::Subscriber<geometry_msgs::Twist>  sub_ptu("/ptu/cmd_vel", &ptu_cb );

void setup()
{
  //Select Pins to be used on Arduino for Servo control
  ptu_pan.attach(11);
  ptu_tilt.attach(12);
  
  //Set the Starting Angle
  ptu_pan.write(int(pan_angle * 180 / Pi));
  ptu_tilt.write(int(tilt_angle * 180 / Pi));

  //Start the RosSerial communcation
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  
  //Subscribe and Publish topics previously defined
  nh.advertise(pub_ptu);
  nh.subscribe(sub_ptu);
}

void loop()
{
  nh.spinOnce();
  
  //Set the Pan & Tilt current angle
  ptu_pan.write(int(pan_angle * 180 / Pi));
  ptu_tilt.write(int(tilt_angle * 180 / Pi));

  //Set up the JointState msg
  ptu_jointstatus.header.stamp = nh.now();
  ptu_jointstatus.header.frame_id = "";
  ptu_jointstatus.name_length = 2;
  ptu_jointstatus.position_length = 2;
  ptu_jointstatus.velocity_length = 2;
  ptu_jointstatus.effort_length = 2;
  ptu_jointstatus.name = joint_name;
  ptu_jointstatus.position = joint_position;
  ptu_jointstatus.velocity = joint_velocity;
  ptu_jointstatus.effort = joint_effort;
  
  //Publish the new jointstate to /ptu/joint_states
  pub_ptu.publish(&ptu_jointstatus);
}

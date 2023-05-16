#include "mobile_robot/mobilerobot_drive.h"

MobileRobot::MobileRobot()
  : nh_priv_("~"){
  //Init gazebo ros turtlebot3 node
  ROS_INFO("Turtlebot initialisiert");
  ROS_ASSERT(init());
}

MobileRobot::~MobileRobot(){
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}


bool MobileRobot::init(){
  //intialize Publisher
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  //intialize Subscriber
  odom_sub = nh.subscribe("/odom_euler", 10, &MobileRobot::odomMsgCallBack, this);
  range_sub = nh.subscribe("/ultrasonic", 10, &MobileRobot::rangeMsgcallBack, this);
  infra_sub = nh.subscribe("/infra", 10, &MobileRobot::infraMsgcallBack, this);
  
  
  brightness = 0;
  
  return true;
}
void MobileRobot::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;
  cmd_vel_pub.publish(cmd_vel);
}

void MobileRobot::rangeMsgcallBack(const std_msgs::Float32MultiArray &msg){
  range_data[0] = msg.data[0];
  range_data[1] = msg.data[1];
  range_data[2] = msg.data[2];
}

void MobileRobot::infraMsgcallBack(const sensor_msgs::Range &msg){
  
  
  if (msg.range >= THRESHOLD){
    brightness = HELL;
  }
  else if (msg.range < THRESHOLD){
    brightness = DUNKEL;
  }
}

void MobileRobot::odomMsgCallBack(const nav_msgs::Odometry &msg){
  
}


bool MobileRobot::controlLoop(){

  return true;

}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "mobilerobot_drive");
  MobileRobot Mobilerobot;

  ros::Rate loop_rate(125);
  bool continueRobot = true;
  while (ros::ok() && continueRobot == true)
  {
    continueRobot = Mobilerobot.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


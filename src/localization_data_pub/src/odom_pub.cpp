#include "localization_data_pub/odom_pub.h"


OdomCalc::OdomCalc()
  : nh_priv_("~"){
  //Init ros odom_pub node
  ROS_INFO("odom_pub initialisiert");
  ROS_ASSERT(init());
}

OdomCalc::~OdomCalc(){
  ros::shutdown();
}

bool OdomCalc::init(){
  distanceLeft = 0;
  distanceRight = 0;
  odomOld.pose.pose.position.x = 0;
  odomOld.pose.pose.position.y = 0;
  odomOld.pose.pose.orientation.z = 0;
  //Initialize Publisher
  odom_data_pub = nh.advertise<nav_msgs::Odometry>("/odom_euler", 100);

  //Initialize Subscriber
  right_counts_sub = nh.subscribe("right_ticks", 100, &OdomCalc::Calc_Right, this);
  left_counts_sub = nh.subscribe("left_ticks", 100, &OdomCalc::Calc_Left, this); 
  return true;
}


// Calculate the distance the left wheel has traveled since the last cycle
void OdomCalc::Calc_Left(const std_msgs::Int16& leftCount){
 
  static int lastCountL = 0;
  if(leftCount.data != 0 && lastCountL != 0) {
         
    int leftTicks = (leftCount.data - lastCountL);
 
    if (leftTicks > 10000) {
      leftTicks = 0 - (65535 - leftTicks);
    }
    else if (leftTicks < -10000) {
      leftTicks = 65535-leftTicks;
    }
    else{}
    distanceLeft = leftTicks*MMperTICK;
  }
  lastCountL = leftCount.data;
}

// Calculate the distance the right wheel has traveled since the last cycle
void OdomCalc::Calc_Right(const std_msgs::Int16& rightCount){
   
  static int lastCountR = 0;
  if(rightCount.data != 0 && lastCountR != 0) {
 
    int rightTicks = rightCount.data - lastCountR;
     
    if (rightTicks > 10000) {
      distanceRight = (0 - (65535 - distanceRight))*MMperTICK;
    }
    else if (rightTicks < -10000) {
      rightTicks = 65535 - rightTicks;
    }
    else{}
    distanceRight = rightTicks*MMperTICK;
  }
}

void OdomCalc::update_odom() {
 
  // Calculate the average distance
  double cycleDistance = (distanceRight + distanceLeft) / 2;
   
  // Calculate the number of radians the robot has turned since the last cycle
  double cycleAngle = asin((distanceRight-distanceLeft)/WHEEL_BASE);
 
  // Average angle during the last cycle
  double avgAngle = cycleAngle/2 + odomOld.pose.pose.orientation.z;
     
  if (avgAngle > PI) {
    avgAngle -= 2*PI;
  }
  else if (avgAngle < -PI) {
    avgAngle += 2*PI;
  }
  else{}
 
  // Calculate the new pose (x, y, and theta)
  odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle)*cycleDistance;
  odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle)*cycleDistance;
  odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;
 
  // Prevent lockup from a single bad cycle
  if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y)
     || isnan(odomNew.pose.pose.position.z)) {
    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
    odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
  }
 
  // Make sure theta stays in the correct range
  if (odomNew.pose.pose.orientation.z > PI) {
    odomNew.pose.pose.orientation.z -= 2 * PI;
  }
  else if (odomNew.pose.pose.orientation.z < -PI) {
    odomNew.pose.pose.orientation.z += 2 * PI;
  }
  else{}
 
  // Compute the velocity
  odomNew.header.stamp = ros::Time::now();
  odomNew.twist.twist.linear.x = cycleDistance/(odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());
  odomNew.twist.twist.angular.z = cycleAngle/(odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());
 
  // Save the pose data for the next cycle
  odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
  odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
  odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
  odomOld.header.stamp = odomNew.header.stamp;
 
  // Publish the odometry message
  odom_data_pub.publish(odomNew);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "odom_pub");
  OdomCalc odomCalc;

  ros::Rate loop_rate(30); 
     
  while(ros::ok()) {
     
  
    odomCalc.update_odom();
      
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
 
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
  initialX = 0.0;
  initialY = 0.0;
  initialTheta = 0.00000000001;
  bool initialPoseRecieved = false;

  odomNew.header.frame_id = "odom";
  odomNew.pose.pose.position.z = 0;
  odomNew.pose.pose.orientation.x = 0;
  odomNew.pose.pose.orientation.y = 0;
  odomNew.twist.twist.linear.x = 0;
  odomNew.twist.twist.linear.y = 0;
  odomNew.twist.twist.linear.z = 0;
  odomNew.twist.twist.angular.x = 0;
  odomNew.twist.twist.angular.y = 0;
  odomNew.twist.twist.angular.z = 0;

  odomOld.pose.pose.position.x = initialX;
  odomOld.pose.pose.position.y = initialY;
  odomOld.pose.pose.orientation.z = initialTheta;
  //Initialize Publisher
  odom_data_pub = nh.advertise<nav_msgs::Odometry>("odom_euler", 100);
  odom_data_pub_quat = nh.advertise<nav_msgs::Odometry>("odom_data_quat", 100);

  //Initialize Subscriber
  right_counts_sub = nh.subscribe("right_ticks", 100, &OdomCalc::Calc_Right, this);
  left_counts_sub = nh.subscribe("left_ticks", 100, &OdomCalc::Calc_Left, this);
  subInitialPose = nh.subscribe("initial_2d", 1, &OdomCalc::set_initial_2d, this); 
  return true;
}

void OdomCalc::set_initial_2d(const geometry_msgs::PoseStamped &rvizClick) {
 
  odomOld.pose.pose.position.x = rvizClick.pose.position.x;
  odomOld.pose.pose.position.y = rvizClick.pose.position.y;
  odomOld.pose.pose.orientation.z = rvizClick.pose.orientation.z;
  initialPoseRecieved = true;
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

// Publish a nav_msgs::Odometry message in quaternion format
void OdomCalc::publish_quat() {
 
  tf2::Quaternion q;
         
  q.setRPY(0, 0, odomNew.pose.pose.orientation.z);
 
  nav_msgs::Odometry quatOdom;
  quatOdom.header.stamp = odomNew.header.stamp;
  quatOdom.header.frame_id = "odom";
  quatOdom.child_frame_id = "base_link";
  quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
  quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
  quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
  quatOdom.pose.pose.orientation.x = q.x();
  quatOdom.pose.pose.orientation.y = q.y();
  quatOdom.pose.pose.orientation.z = q.z();
  quatOdom.pose.pose.orientation.w = q.w();
  quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
  quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
  quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
  quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
  quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
  quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;
 
  for(int i = 0; i<36; i++) {
    if(i == 0 || i == 7 || i == 14) {
      quatOdom.pose.covariance[i] = .01;
     }
     else if (i == 21 || i == 28 || i== 35) {
       quatOdom.pose.covariance[i] += 0.1;
     }
     else {
       quatOdom.pose.covariance[i] = 0;
     }
  }
 
  odom_data_pub_quat.publish(quatOdom);
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
    if(odomCalc.initialPoseRecieved){
      odomCalc.update_odom();
      odomCalc.publish_quat();
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
 
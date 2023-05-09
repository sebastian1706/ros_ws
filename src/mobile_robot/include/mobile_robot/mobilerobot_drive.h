#ifndef MOBILEROBOT_DRIVE_H_
#define MOBILEROBOT_DRIVE_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>

#define THRESHOLD 350
#define HELL 1
#define DUNKEL 2



class MobileRobot{
 public:
    MobileRobot();
    ~MobileRobot();
    bool init();
    bool controlLoop();

 private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv_;
    //Variables
    //Ultrasonic range
    double scanData;
    int brightness;

    //ROS Topic Publisher
    ros::Publisher cmd_vel_pub;

    //ROS Topic Subscriber
    ros::Subscriber odom_sub;
    ros::Subscriber range_sub;
    ros::Subscriber infra_sub;

    //Functions
    void odomMsgCallBack(const nav_msgs::Odometry &msg);
    void infraMsgcallBack(const sensor_msgs::Range &msg);
    void rangeMsgcallBack(const sensor_msgs::Range &msg);
    void updatecommandVelocity(double linear, double angular);
    
};

#endif //MOBILEROBOT_DRIVE_H_
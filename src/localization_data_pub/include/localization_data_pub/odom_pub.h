#ifndef ODOM_PUB_H_
#define ODOM_PUB_H_

#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#define MMperTICK  10.21
#define PI 3.14
#define WHEEL_BASE 0.13

class OdomCalc{
    public:
        OdomCalc();
        ~OdomCalc();
        bool init();
        void update_odom();
        void publish_quat();
        bool initialPoseRecieved;
    
    private:
        ros::NodeHandle nh;
        ros::NodeHandle nh_priv_;
        // Distance both wheels have traveled
        double distanceLeft;
        double distanceRight;
        double initialX;
        double initialY;
        double initialTheta;
        
        void set_initial_2d(const geometry_msgs::PoseStamped &rvizClick);
        nav_msgs::Odometry odomNew;
        nav_msgs::Odometry odomOld;

        //Ros Topic Publisher   
        ros::Publisher odom_data_pub;
        ros::Publisher odom_data_pub_quat;

        //Ros Topic Subscriber
        ros::Subscriber right_counts_sub;
        ros::Subscriber left_counts_sub;
        ros::Subscriber subInitialPose;
        
        //Initialize functions
        void Calc_Left(const std_msgs::Int16 &leftCount);
        void Calc_Right(const std_msgs::Int16 &leftCount);
};


#endif
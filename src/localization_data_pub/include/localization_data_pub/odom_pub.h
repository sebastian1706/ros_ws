#ifndef ODOM_PUB_H_
#define ODOM_PUB_H_

#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include <nav_msgs/Odometry.h>

#define MMperTICK  10.21
#define PI 3.14
#define WHEEL_BASE 0.13

class OdomCalc{
    public:
        OdomCalc();
        ~OdomCalc();
        bool init();
    
    private:
        ros::NodeHandle nh;
        ros::NodeHandle nh_priv_;
        // Distance both wheels have traveled
        double distanceLeft;
        double distanceRight;
        nav_msgs::Odometry odomNew;
        nav_msgs::Odometry odomOld;

        //Ros Topic Publisher   
        ros::Publisher odom_data_pub;

        //Ros Topic Subscriber
        ros::Subscriber right_counts_sub;
        ros::Subscriber left_counts_sub;
        
        //Initialize functions
        void Calc_Left(const std_msgs::Int16 &leftCount);
        void Calc_Right(const std_msgs::Int16 &leftCount);
        void update_odom();

};


#endif
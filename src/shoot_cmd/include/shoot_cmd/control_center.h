#ifndef CONTROL_CENTER_H
#define CONTROL_CENTER_H

#include "ros/ros.h"
#include <math.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>

using namespace std;

class ControlCenter{

public:
    ControlCenter(ros::NodeHandle& nh):nn(nh){}
    ~ControlCenter(){}
    void initROSModule();

private:

    int target_id;
    Eigen::Vector2f A,B,C,D,E;
    float A_yaw, B_yaw, C_yaw, D_yaw, E_yaw;
    Eigen::Vector3d base_pos;
    actionlib_msgs::GoalStatusArray movebase_state;
    tf::TransformListener pos_listener;
    bool reach_sign;

    bool start_move;
    bool pub_A, pub_B, pub_C, pub_D, pub_E;

    ros::NodeHandle nn;
    ros::Subscriber status_sub, ar_sub, pos_sub, odom_sub, voice_sub;
    ros::Publisher voice_pub;
    ros::Timer exec_timer_;

    float quaternion_to_yaw(const Eigen::Quaterniond &q);
    Eigen::Quaterniond quaternion_from_euler(float roll, float pitch, float yaw);
    void status_cb(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);
    void pos_cb(const tf2_msgs::TFMessage::ConstPtr &msg);
    void odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void voice_cb(const std_msgs::String::ConstPtr &msg);
    void ar_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
    float satfunc(float data, float Max);
    void publishYawvel(float dyaw);
    void publishStop();
    
    void execCallback(const ros::TimerEvent& e);
    bool reach_grab_point();
};

#endif

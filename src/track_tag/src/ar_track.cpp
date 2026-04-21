#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <iostream>
#include <string>

using namespace std;

geometry_msgs::Twist Command_now;
ar_track_alvar_msgs::AlvarMarker marker;

float distance_thres;
float kpx_track;
float kpz_track;

float track_max_vel_x;
float track_thres_vel_x;

float track_max_vel_z;
float track_thres_vel_z;

void printf_param();
void printf_result();
float satfunc(float data, float Max, float Thres); 

float vel_x = 0.0; //linear-vel-x
float vel_z = 0.0; //angular-vel-z

float offset_x = 0.0; //left-right-pose
float offset_z = 0.0; //forward-backward-pose
int track_flag;
int arrive_flag = 1;
int count_sum = 0;

void tag_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
    if (msg->markers.size() == 1)
    {
        marker = msg->markers[0];
        offset_x = marker.pose.pose.position.x;
        offset_z = marker.pose.pose.position.z - distance_thres;
        track_flag = 1;

        vel_x = kpx_track * offset_z;
        vel_z = -kpz_track * offset_x;
        satfunc(vel_x, track_max_vel_x, track_thres_vel_x);
        satfunc(vel_z, track_max_vel_z, track_thres_vel_z);
    }
    else
    {
        track_flag = 0;
    }
}

void arrive_cb(const std_msgs::String::ConstPtr &msg)
{
    arrive_flag = 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ar_track");
    ros::NodeHandle nh;

    ros::Subscriber tag_sub = nh.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 10, tag_cb);
    ros::Subscriber arrive_sub = nh.subscribe<std_msgs::String>("/mission/arrived", 10, arrive_cb);

    ros::Publisher command_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    ros::Publisher shoot_pub = nh.advertise<std_msgs::String>("/shoot", 100);
    ros::Rate rate(20.0);

    nh.param<float>("kpx_track", kpx_track, 1.0);
    nh.param<float>("kpz_track", kpz_track, 8.0);

    nh.param<float>("track_max_vel_x", track_max_vel_x, 0.5); 
    nh.param<float>("track_max_vel_z", track_max_vel_z, 0.5); 
    nh.param<float>("track_thres_vel_x", track_thres_vel_x, 0.05);    
    nh.param<float>("track_thres_vel_z", track_thres_vel_z, 0.05);
    nh.param<float>("distance_thres", distance_thres, 0.5);
    printf_param();

    int check_flag;
    cout << "Please check the parameter and setting，1 for go on， else for quit: "<<endl;
    cin >> check_flag;

    while(ros::ok())
    {
        ros::spinOnce();
        printf_result();

        if(track_flag == 0 || arrive_flag == 0)
        {
            Command_now.linear.x = 0.0;
            Command_now.angular.z = 0.0;
        }
        else
        {   
            Command_now.linear.x = 0.0;
            Command_now.angular.z = vel_z;
        }
        command_pub.publish(Command_now);

        if (offset_x<0.2 && offset_x>-0.2)
        { 
          if (count_sum <= 10)
          {
              count_sum += 1;
              std_msgs::String shoot_msg;
              shoot_msg.data = string("1");
              shoot_pub.publish(shoot_msg);
              
          }
          track_flag = 0;
          //if (offset_z<0.1 && offset_z>-0.1)
          //{   
          //}
        }
        rate.sleep();
    }
}

//饱和函数
float satfunc(float data, float Max, float Thres)
{
    if (abs(data)<Thres)
    {
        return 0;
    }
    else if(abs(data)>Max)
    {
        return ( data > 0 ) ? Max : -Max;
    }
    else
    {
        return data;
    }
}

void printf_result()
{
    cout.setf(ios::fixed);
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "track_tag: " <<  track_flag <<endl;

    cout << "pos_target: [X Z] : " << " " << offset_x  << " [m] " << offset_z <<" [m] "<<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>Control State<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Command: " << Command_now.linear.x << " [m/s] "<< Command_now.angular.z << " [m/s] "<<endl;
}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "distance_thres : "<< distance_thres << endl;
    cout << "kpx_track : "<< kpx_track << endl;
    cout << "kpz_track : "<< kpz_track << endl;
    cout << "track_max_vel_x : "<< track_max_vel_x << endl;
    cout << "track_max_vel_z : "<< track_max_vel_z << endl;
    cout << "track_thres_vel_x : "<< track_thres_vel_x << endl;
    cout << "track_thres_vel_z : "<< track_thres_vel_z << endl;
}

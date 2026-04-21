#ifndef CAM_TRACK_NODE_H
#define CAM_TRACK_NODE_H
#include <ros/ros.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>


using namespace std;
namespace cam_track {

class CamTrack{
public:
    CamTrack(ros::NodeHandle& nh):nn(nh){}
    ~CamTrack(){}
    void initROSModule()//设置订阅器和发布器
    {
        nn.param<float>("PID_Control_P",PID_Control_P,0.5);
        nn.param<float>("Max_yaw_vel",Max_yaw_vel,0.2);
        nn.param<float>("yaw_th",yaw_th,0.02);
        nn.param<float>("A_x",A[0],1.5);
        nn.param<float>("A_y",A[1],-0.53);
        nn.param<float>("B_x",B[0],1.5);
        nn.param<float>("B_y",B[1],-2.2);
        nn.param<float>("vel_th",vel_th,0.001);

        status_sub = nn.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status",10,&CamTrack::status_cb,this);//move base status
        pos_sub = nn.subscribe<tf2_msgs::TFMessage>("/tf",1000,&CamTrack::pos_cb,this);
        ar_sub = nn.subscribe<tf2_msgs::TFMessage>("/tf",1000,&CamTrack::ar_cb,this);
        odom_sub = nn.subscribe<nav_msgs::Odometry>("/odom",10,&CamTrack::odom_cb,this);

        cmd_pub = nn.advertise<geometry_msgs::Twist>("/cmd_vel",10);
        goal_pub = nn.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);
        shoot_pub = nn.advertise<std_msgs::String>("/shoot",10);

        exec_timer_ = nn.createTimer(ros::Duration(0.05),&CamTrack::execCallback,this);
    }
    float PID_Control_P;
    float Max_yaw_vel;
    float yaw_th;
    float vel_th;
    Eigen::Vector2f A,B;
    Eigen::Vector3d ar0_pos,ar1_pos,ar2_pos,ar3_pos,base_pos;
    float angle_vel;
    float base_yaw;
    actionlib_msgs::GoalStatusArray movebase_state;
    tf::TransformListener ar_listener,pos_listener;
    bool reach_sign;


private:
    ros::NodeHandle nn;
    ros::Subscriber status_sub,ar_sub,pos_sub,odom_sub;
    ros::Publisher cmd_pub,goal_pub,shoot_pub;
    ros::Timer exec_timer_;
    float quaternion_to_yaw(const Eigen::Quaterniond &q)
    {
        float quat[4];
        quat[0] = q.w();
        quat[1] = q.x();
        quat[2] = q.y();
        quat[3] = q.z();

        Eigen::Vector3d ans;
        ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
        ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
        ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
        return ans[2];
    }
    void status_cb(const actionlib_msgs::GoalStatusArray::ConstPtr &msg){
        movebase_state = *msg;
        if(movebase_state.status_list.size() == 0 ) return;
        if(movebase_state.status_list[0].status == 3)
        {
            reach_sign = true;
            //cout<<"reach destination!"<<endl;
        }
        else reach_sign = false;
    }
    void ar_cb(const tf2_msgs::TFMessage::ConstPtr &msg){
        tf::StampedTransform transform;
        if (msg->transforms[0].child_frame_id == "ar_marker_0"){
            try{
                ar_listener.lookupTransform("map","ar_marker_0",ros::Time(0),transform);
            }
            catch (tf::TransformException &ex)
            {
                ROS_INFO("Couldn't get transform");
                ROS_WARN("%s",ex.what());
                return;
            }
            ar0_pos[0] = transform.getOrigin().x();
            ar0_pos[1] = transform.getOrigin().y();
            ar0_pos[2] = transform.getOrigin().z();
            //cout<<"-------------------------------------------------------------------"<<endl;
            //cout<<"ar0: "<<ar0_pos[0]<<"  "<<ar0_pos[1]<<"  "<<ar0_pos[2]<<endl;

        }
        else if(msg->transforms[0].child_frame_id == "ar_marker_1"){
            try{
                ar_listener.lookupTransform("map","ar_marker_1",ros::Time(0),transform);
            }
            catch (tf::TransformException &ex)
            {
                ROS_INFO("Couldn't get transform");
                ROS_WARN("%s",ex.what());
                return;
            }
            ar1_pos[0] = transform.getOrigin().x();
            ar1_pos[1] = transform.getOrigin().y();
            ar1_pos[2] = transform.getOrigin().z();
            //cout<<"-------------------------------------------------------------------"<<endl;
            //cout<<"ar1: "<<ar1_pos[0]<<"  "<<ar1_pos[1]<<"  "<<ar1_pos[2]<<endl;

        }
        else if(msg->transforms[0].child_frame_id == "ar_marker_2"){
            try{
                ar_listener.lookupTransform("map","ar_marker_2",ros::Time(0),transform);
            }
            catch (tf::TransformException &ex)
            {
                ROS_INFO("Couldn't get transform");
                ROS_WARN("%s",ex.what());
                return;
            }
            ar2_pos[0] = transform.getOrigin().x();
            ar2_pos[1] = transform.getOrigin().y();
            ar2_pos[2] = transform.getOrigin().z();
            //cout<<"-------------------------------------------------------------------"<<endl;
            //cout<<"ar2: "<<ar2_pos[0]<<"  "<<ar2_pos[1]<<"  "<<ar2_pos[2]<<endl;

        }
        else if(msg->transforms[0].child_frame_id == "ar_marker_3"){
            try{
                ar_listener.lookupTransform("map","ar_marker_3",ros::Time(0),transform);
            }
            catch (tf::TransformException &ex)
            {
                ROS_INFO("Couldn't get transform");
                ROS_WARN("%s",ex.what());
                return;
            }
            ar3_pos[0] = transform.getOrigin().x();
            ar3_pos[1] = transform.getOrigin().y();
            ar3_pos[2] = transform.getOrigin().z();
            //cout<<"-------------------------------------------------------------------"<<endl;
            //cout<<"ar3: "<<ar3_pos[0]<<"  "<<ar3_pos[1]<<"  "<<ar3_pos[2]<<endl;

        }
    }
    void pos_cb(const tf2_msgs::TFMessage::ConstPtr &msg){
        tf::StampedTransform transform;
        try{
            ar_listener.lookupTransform("map","base",ros::Time(0),transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_INFO("Couldn't get transform");
            ROS_WARN("%s",ex.what());
            return;
        }
        base_pos[0] = transform.getOrigin().x();
        base_pos[1] = transform.getOrigin().y();
        base_pos[2] = transform.getOrigin().z();
        Eigen::Quaterniond q;
        tf::quaternionTFToEigen(transform.getRotation(),q);
        base_yaw = quaternion_to_yaw(q);
    }
    void odom_cb(const nav_msgs::Odometry::ConstPtr &msg){
        angle_vel = msg->twist.twist.angular.z;
    }
    float calaryaw(int i){
        float yaw;
        Eigen::Vector3d ar_pos;
        if(i==0)
        {
            ar_pos = ar0_pos;
        }
        else if(i==1)
        {
            ar_pos = ar1_pos;
        }
        else if(i==2)
        {
            ar_pos = ar2_pos;
        }
        else if(i==3)
        {
            ar_pos = ar3_pos;
        }
        yaw = atan2(ar_pos[1]-base_pos[1],ar_pos[0]-base_pos[0]);
        return yaw;
    }
    float satfunc(float data, float Max)
    {
        if(abs(data)>Max)
        {
            return ( data > 0 ) ? Max : -Max;
        }
        else
        {
            return data;
        }
    }
    void publishYawvel(float dyaw){
        geometry_msgs::Twist cmd_vel;
        float yaw_vel =  satfunc(PID_Control_P*dyaw,Max_yaw_vel);
        cout<<"Max: "<<Max_yaw_vel<<endl;
        cout<<"~~~~~~~~~"<<endl;
        cout<<"yaw vel: "<<yaw_vel*180/M_PI<<endl;
        cmd_vel.angular.z = yaw_vel;
        cmd_pub.publish(cmd_vel);
    }
    void publishStop(){
        geometry_msgs::Twist cmd_vel;
        cmd_pub.publish(cmd_vel);
    }
    void pointToar(int i){
        float des_yaw;
        des_yaw = calaryaw(i);
        cout<<"des_yaw: "<<des_yaw*180/M_PI<<endl;
        float dyaw = des_yaw - base_yaw;
        cout<<"dyaw: "<<dyaw*180/M_PI<<endl;
        cout<<"angle_vel: "<<abs(angle_vel)*180/M_PI<<endl;

        if(abs(dyaw) > yaw_th)
            publishYawvel(dyaw);
        else
        {
            publishStop();
            //TODO:odom twist angle z < 0.000x
            if(abs(angle_vel)<vel_th)
            {
                std_msgs::String data;
                data.data = "shoot!";
                shoot_pub.publish(data);
                if(pub_A&&!pub_B) shoot_A = true;
                else if(pub_B) shoot_B[i-1] = true;
            }

        }
    }

    bool pub_A,pub_B;
    bool shoot_A,shoot_B[3];
    void execCallback(const ros::TimerEvent& e){
        if(!pub_A&&!shoot_A){
            geometry_msgs::PoseStamped A_goal;
            A_goal.header.frame_id = "map";
            A_goal.header.stamp = ros::Time::now();
            A_goal.pose.position.x = A[0];
            A_goal.pose.position.y = A[1];
            A_goal.pose.orientation.x = 0;
            A_goal.pose.orientation.y = 0;
            A_goal.pose.orientation.z = 0;
            A_goal.pose.orientation.w = 1.0;

            goal_pub.publish(A_goal);

            pub_A = true;
        }
        if(shoot_A&&!pub_B){
            geometry_msgs::PoseStamped B_goal;
            B_goal.header.frame_id = "map";
            B_goal.header.stamp = ros::Time::now();
            B_goal.pose.position.x = B[0];
            B_goal.pose.position.y = B[1];
            B_goal.pose.orientation.x = 0;
            B_goal.pose.orientation.y = 0;
            B_goal.pose.orientation.z = 0;
            B_goal.pose.orientation.w = 1.0;

            goal_pub.publish(B_goal);

            pub_B = true;
        }
        if(!reach_sign) return;
        cout<<"==============================="<<endl;
        cout<<"base yaw: "<<base_yaw*180/M_PI<<endl;
        if(!shoot_A){
            pointToar(0);
        }
        else if(!shoot_B[0])
        {
            pointToar(1);
        }
        else if(!shoot_B[1])
        {
            pointToar(2);
        }
        else if(!shoot_B[2])
        {
            pointToar(3);
        }
    }
};
}
#endif

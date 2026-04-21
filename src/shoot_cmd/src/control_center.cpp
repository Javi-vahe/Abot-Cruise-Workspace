#include "../include/shoot_cmd/control_center.h"
#include <cmath>


void ControlCenter::initROSModule()
{
    nn.param<int>("target_id", target_id, 0);

    nn.param<float>("A_x", A[0], 0.0);
    nn.param<float>("A_y", A[1], 0.0);
    nn.param<float>("A_yaw", A_yaw, 0.0);
    nn.param<float>("B_x", A[0], 0.0);
    nn.param<float>("B_y", A[1], 0.5);
    nn.param<float>("B_yaw", A_yaw, 0.0);
    nn.param<float>("C_x", A[0], 0.0);
    nn.param<float>("C_y", A[1], 0.0);
    nn.param<float>("C_yaw", A_yaw, 0.0);
    nn.param<float>("D_x", B[0], 0.0);
    nn.param<float>("D_y", B[1], -2.8);
    nn.param<float>("D_yaw", B_yaw, 0.0);
    nn.param<float>("E_x", B[0], 0.0);
    nn.param<float>("E_y", B[1], -2.8);
    nn.param<float>("E_yaw", B_yaw, 0.0);

    start_move = false;
    pub_A = pub_B = pub_C = pub_D = pub_E = false;

    status_sub = nn.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status",10,&ControlCenter::status_cb,this);
    pos_sub = nn.subscribe<tf2_msgs::TFMessage>("/tf",1000,&ControlCenter::pos_cb,this);
    voice_sub = nn.subscribe<std_msgs::String>("/snowman/ask", 100, &ControlCenter::voice_cb, this);
    ar_sub = nn.subscribe<geometry_msgs::Point>("/ar_pose_marker", 10, &ControlCenter::ar_cb, this);

    cmd_pub = nn.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    goal_pub = nn.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10); 
    exec_timer_ = nn.createTimer(ros::Duration(0.05), &ControlCenter::execCallback, this);
}

float ControlCenter::quaternion_to_yaw(const Eigen::Quaterniond &q)
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

Eigen::Quaterniond ControlCenter::quaternion_from_euler(float roll, float pitch, float yaw)
{
    Eigen::Quaterniond q;
    geometry_msgs::Quaternion qt;
    qt = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
    q.w() = qt.w;
    q.x() = qt.x;
    q.y() = qt.y;
    q.z() = qt.z;
    return q;
}

void ControlCenter::status_cb(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
    movebase_state = *msg;
    if(movebase_state.status_list.size() == 0 ) return;
    if(movebase_state.status_list[0].status == 3)
    {
        reach_sign = true;
    }
    else reach_sign = false;
}

void ControlCenter::voice_cb(const std_msgs::String::ConstPtr &msg)
{   
    /*
    std::string dataString = msg->data;
    if(dataString.compare("开始导航。") == 0)
    {
        start_move = true;
    } 
    */
    start_move = true;   
}

void ControlCenter::ar_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
    if (msg->markers.size() == 1)
    {
        marker = msg->markers[0];
    }  
}

void ControlCenter::pos_cb(const tf2_msgs::TFMessage::ConstPtr &msg)
{
    tf::StampedTransform transform;
    try{
        pos_listener.lookupTransform("map","base_link",ros::Time(0),transform);
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
    //cout<<"X: "<<base_pos[0]<<" Y: "<<base_pos[1]<<" Yaw: "<<base_yaw<<endl;
}

void ControlCenter::odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    angle_vel = msg->twist.twist.angular.z;
}

float ControlCenter::satfunc(float data, float Max)
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

void ControlCenter::publishYawvel(float dyaw)
{
    geometry_msgs::Twist cmd_vel;
    float yaw_vel =  satfunc(PID_Control_P*dyaw,Max_yaw_vel);
    //cout<<"Max: "<<Max_yaw_vel<<endl;
    //cout<<"~~~~~~~~~"<<endl;
    //cout<<"yaw vel: "<<yaw_vel*180/M_PI<<endl;
    cmd_vel.angular.z = yaw_vel;
    cmd_pub.publish(cmd_vel);
}

void ControlCenter::publishStop()
{
    geometry_msgs::Twist cmd_vel;
    cmd_pub.publish(cmd_vel);
}

void ControlCenter::execCallback(const ros::TimerEvent& e)
{   
    if (!start_move) return;
    if (!pub_A)
    {  
        ROS_INFO("Moving to A point at X: %f, Y:%f, Yaw: %f......", A[0], A[1], A_yaw);
        geometry_msgs::PoseStamped A_goal;
        A_goal.header.frame_id = "map";
        A_goal.header.stamp = ros::Time::now();
        A_goal.pose.position.x = A[0];
        A_goal.pose.position.y = A[1];
        Eigen::Quaterniond A_q = quaternion_from_euler(0.0, 0.0, A_yaw);
        A_goal.pose.orientation.x = A_q.x();
        A_goal.pose.orientation.y = A_q.y();
        A_goal.pose.orientation.z = A_q.z();
        A_goal.pose.orientation.w = A_q.w();
        goal_pub.publish(A_goal);
        pub_A = true;
    }
    if (!pub_B)
    {
        ROS_INFO("Moving to B point at X: %f, Y:%f, Yaw: %f......", B[0], B[1], B_yaw);
        geometry_msgs::PoseStamped B_goal;
        B_goal.header.frame_id = "map";
        B_goal.header.stamp = ros::Time::now();
        B_goal.pose.position.x = B[0];
        B_goal.pose.position.y = B[1];
        Eigen::Quaterniond B_q = quaternion_from_euler(0.0, 0.0, B_yaw);
        B_goal.pose.orientation.x = B_q.x();
        B_goal.pose.orientation.y = B_q.y();
        B_goal.pose.orientation.z = B_q.z();
        B_goal.pose.orientation.w = B_q.w();
        goal_pub.publish(B_goal);
        pub_B = true;        
    }
    if (!pub_C)
    {
        ROS_INFO("Moving to C point at X: %f, Y:%f, Yaw: %f......", C[0], C[1], C_yaw);
        geometry_msgs::PoseStamped C_goal;
        B_goal.header.frame_id = "map";
        B_goal.header.stamp = ros::Time::now();
        B_goal.pose.position.x = C[0];
        B_goal.pose.position.y = C[1];
        Eigen::Quaterniond C_q = quaternion_from_euler(0.0, 0.0, C_yaw);
        C_goal.pose.orientation.x = C_q.x();
        C_goal.pose.orientation.y = C_q.y();
        C_goal.pose.orientation.z = C_q.z();
        C_goal.pose.orientation.w = C_q.w();
        goal_pub.publish(C_goal);
        pub_C = true;        
    }  
    if (!pub_D)
    {
        ROS_INFO("Moving to D point at X: %f, Y:%f, Yaw: %f......", D[0], D[1], D_yaw);
        geometry_msgs::PoseStamped D_goal;
        D_goal.header.frame_id = "map";
        D_goal.header.stamp = ros::Time::now();
        D_goal.pose.position.x = C[0];
        D_goal.pose.position.y = C[1];
        Eigen::Quaterniond C_q = quaternion_from_euler(0.0, 0.0, C_yaw);
        D_goal.pose.orientation.x = D_q.x();
        D_goal.pose.orientation.y = D_q.y();
        D_goal.pose.orientation.z = D_q.z();
        D_goal.pose.orientation.w = D_q.w();
        goal_pub.publish(D_goal);
        pub_D = true;        
    }  
    if (!reach_sign) return;
}


int main(int argc, char** argv)
{
    ros::init(argc,argv,"control_center");
    ros::NodeHandle nh("~");
    ControlCenter control_center(nh);
    control_center.initROSModule();
    ros::spin();
    return 0;

}

#include <iostream>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "robot_slam/Adjust.h"

class Adjust_Server
{
private:
    ros::NodeHandle nh_;
    ros::ServiceServer service_;
    ros::Publisher cmd_vel_puber, pub_is_adjusting;

    tf::TransformListener listener_;
    geometry_msgs::Pose2D now_pose, goal_pose;
    geometry_msgs::Twist twist;

    int loop_rate = 20;
    int timeout = 6;
    double xy_tolerance = 0.1;
    double yaw_tolerance = 0.1;
    double base_linear = 2;
    double base_angular = 3;
    bool debug = false;

public:
    Adjust_Server()
    {
        nh_ = ros::NodeHandle("~");
        service_ = nh_.advertiseService("/adjust", &Adjust_Server::adjust_cb, this);
        cmd_vel_puber = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_original", 10);
        pub_is_adjusting = nh_.advertise<std_msgs::Bool>("/is_adjusting", 1);

        nh_.param("loop_rate", loop_rate, 15);
        nh_.param("timeout", timeout, 6);
        nh_.param("xy_tolerance", xy_tolerance, 0.1);
        nh_.param("yaw_tolerance", yaw_tolerance, 0.1);
        nh_.param("base_linear", base_linear, 2.0);
        nh_.param("base_angular", base_angular, 3.0);
        nh_.param("debug", debug, false);

        ROS_INFO("looprate: %dHz", loop_rate);
        ROS_INFO("timeout: %ds", timeout);
        ROS_INFO("xy_tolerance: %fm", xy_tolerance);
        ROS_INFO("yaw_tolerance: %fm", yaw_tolerance);
        ROS_INFO("base_linear: %fm/s", base_linear);
        ROS_INFO("base_angular: %frad/s", base_angular);
        ROS_INFO("debug: %d", debug);

        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.angular.z = 0;
        ROS_INFO("Service 'adjust' is ready.");
    }

    bool adjust_cb(robot_slam::Adjust::Request &req, robot_slam::Adjust::Response &res)
    {
        std_msgs::Bool is_adjusting;
        is_adjusting.data = true;
        ROS_WARN("received adjust request: x=%f y=%f yaw=%f", req.x, req.y, req.yaw);
        goal_pose.x = req.x;
        goal_pose.y = req.y;
        goal_pose.theta = req.yaw;
        pub_is_adjusting.publish(is_adjusting);
        res.success = loop_adjust(goal_pose);
        is_adjusting.data = false;
        pub_is_adjusting.publish(is_adjusting);
        return res.success;
    }

    bool loop_adjust(geometry_msgs::Pose2D goal_pose_)
    {
        ros::Rate r(loop_rate);
        tf::StampedTransform transform;
        double start_time = ros::Time::now().toSec();
        while (ros::ok())
        {
            // debug 比赛时请注释或将debug参数改为false
            if (debug)
            {
                nh_.getParam("timeout", timeout);
                nh_.getParam("xy_tolerance", xy_tolerance);
                nh_.getParam("yaw_tolerance", yaw_tolerance);
                nh_.getParam("base_linear", base_linear);
                nh_.getParam("base_angular", base_angular);
                nh_.getParam("debug", debug);
            } // debug

            try
            {
                listener_.lookupTransform("map", "base_link", ros::Time(0), transform);
                tf::Quaternion q = transform.getRotation();
                double roll, pitch, yaw;
                tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
                now_pose.x = transform.getOrigin().x();
                now_pose.y = transform.getOrigin().y();
                now_pose.theta = yaw;
                ROS_INFO("x: %f, y: %f, yaw: %f", now_pose.x, now_pose.y, yaw);
                // 超时且在tolerance之外返回失败
                if (ros::Time::now().toSec() - start_time >= timeout && !in_tolerance(now_pose, goal_pose))
                {
                    pub_cmd_vel(0, 0, 0);
                    ROS_ERROR("!!!!!!!!!! Adjust time out !!!!!!!!!!");
                    return false;
                }
                // 在tolerance之内返回成功
                if (in_tolerance(now_pose, goal_pose))
                {
                    double begin_time = ros::Time::now().toSec();
                    double now_time = ros::Time::now().toSec();
                    while (ros::ok() && in_tolerance(now_pose, goal_pose) && now_time - begin_time <= 1)
                    { // 1 second delay
                        pub_cmd_vel(0, 0, 0);
                        try
                        {
                            listener_.lookupTransform("map", "base_link", ros::Time(0), transform);
                            tf::Quaternion q = transform.getRotation();
                            double roll, pitch, yaw;
                            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
                            now_pose.x = transform.getOrigin().x();
                            now_pose.y = transform.getOrigin().y();
                            now_pose.theta = yaw;
                        }
                        catch (tf::TransformException &ex)
                        {
                            ROS_ERROR("(1s countdown)Failed to listen TF: %s", ex.what());
                        }
                        now_time = ros::Time::now().toSec();
                    }
                    if (now_time - begin_time >= 1)
                    {
                        ROS_WARN("********** Adjust success **********");
                        return true;
                    }
                }
                // 调整
                double delta_x = goal_pose.x - now_pose.x;
                double delta_y = goal_pose.y - now_pose.y;
                double delta_yaw = goal_pose.theta - now_pose.theta;
                normalizeAngle(delta_yaw);
                ROS_INFO("delta_x: %f, delta_y: %f, delta_yaw: %f", delta_x, delta_y, delta_yaw);
                // x
                if (fabs(delta_x) > xy_tolerance)
                    twist.linear.x = delta_x * base_linear;
                else
                    twist.linear.x = 0;
                // y
                if (fabs(delta_y) > xy_tolerance)
                    twist.linear.y = delta_y * base_linear;
                else
                    twist.linear.y = 0;
                // z
                if (fabs(delta_yaw) > yaw_tolerance)
                    twist.angular.z = delta_yaw * base_angular;
                else
                    twist.angular.z = 0;

                twist_transform(twist.linear.x, twist.linear.y);
                cmd_vel_puber.publish(twist);
                ROS_INFO("pub twist x: %f, y: %f, z: %f", twist.linear.x, twist.linear.y, twist.angular.z);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("Failed to listen TF: %s", ex.what());
            }
            // 防止try listen失败，超时退出
            if (ros::Time::now().toSec() - start_time >= timeout && !in_tolerance(now_pose, goal_pose))
            {
                pub_cmd_vel(0, 0, 0);
                ROS_ERROR("!!!!!!!!!! Adjust time out !!!!!!!!!!");
                return false;
            }
            r.sleep();
        } // while
    } // adjust_cb

    void twist_transform(double &x_, double &y_)
    {
        double origin_x = x_;
        double origin_y = y_;
        x_ = origin_x * cos(-now_pose.theta) - origin_y * sin(-now_pose.theta);
        y_ = origin_x * sin(-now_pose.theta) + origin_y * cos(-now_pose.theta);
    }

    void pub_cmd_vel(auto linear_x, auto linear_y, auto angular_z)
    {
        twist.linear.x = linear_x;
        twist.linear.y = linear_y;
        twist.angular.z = angular_z;
        cmd_vel_puber.publish(twist);
    }

    bool xy_in_tolerance(geometry_msgs::Pose2D current_pose_, geometry_msgs::Pose2D target_pose_)
    {
        double delta_x = fabs(target_pose_.x - current_pose_.x);
        double delta_y = fabs(target_pose_.y - current_pose_.y);
        double dist = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
        return dist <= xy_tolerance;
    }

    bool yaw_in_tolerance(geometry_msgs::Pose2D current_pose_, geometry_msgs::Pose2D target_pose_)
    {
        double delta_yaw = fabs(target_pose_.theta - current_pose_.theta);
        normalizeAngle(delta_yaw);
        return delta_yaw <= yaw_tolerance;
    }

    bool in_tolerance(geometry_msgs::Pose2D current_pose_, geometry_msgs::Pose2D target_pose_)
    {
        double delta_x = fabs(target_pose_.x - current_pose_.x);
        double delta_y = fabs(target_pose_.y - current_pose_.y);
        double delta_yaw = fabs(target_pose_.theta - current_pose_.theta);
        normalizeAngle(delta_yaw);
        double dist = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
        return (dist <= xy_tolerance) && (delta_yaw <= yaw_tolerance);
    }

    // 归一化弧度到 [-π, π]
    void normalizeAngle(double &angle)
    {
        // 使用 fmod 来处理取模操作
        angle = fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0)
        {
            angle += 2.0 * M_PI;
        }
        angle = angle - M_PI;
    }
};

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "adjust_server");
    // 创建服务端对象（初始化服务）
    Adjust_Server server;
    ros::spin();
    return 0;
}

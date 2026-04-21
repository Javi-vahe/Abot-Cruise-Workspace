#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_listener.h>

class Cmd_vel_ralay
{
private:
    ros::Timer timer_;
    ros::Subscriber sub_cmd_vel111;
    ros::Subscriber sub_cmd_vel_factor;
    ros::Subscriber sub_target_pose, sub_is_adjusting;
    ros::Publisher pub_cmd_vel;
    geometry_msgs::Twist twist;
    geometry_msgs::Pose2D target_pose, now_pose;
    tf::TransformListener listener_;
    tf::StampedTransform map_base_transform;
    double vel_factor, freq;
    double yaw_tolerance = 0.1;
    double base_angular = 3;
    bool is_adjusting;
    int zero_times = 0;

public:
    Cmd_vel_ralay(ros::NodeHandle *nh)
    {
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;
        vel_factor = 1;
        freq = 1 / 15;
        is_adjusting = false;
        sub_cmd_vel111 = nh->subscribe("cmd_vel_original", 10, &Cmd_vel_ralay::cmd_vel111_callback, this);
        sub_cmd_vel_factor = nh->subscribe("cmd_vel_factor", 10, &Cmd_vel_ralay::vel_factor_callback, this);
        sub_target_pose = nh->subscribe("target_pose", 1, &Cmd_vel_ralay::target_pose_callback, this);
        sub_is_adjusting = nh->subscribe("is_adjusting", 1, &Cmd_vel_ralay::is_adjusting_callback, this);
        pub_cmd_vel = nh->advertise<geometry_msgs::Twist>("cmd_vel", 10);
        timer_ = nh->createTimer(ros::Duration(freq), &Cmd_vel_ralay::timerCallback, this);
        ROS_INFO("Cmd_vel_ralay node initialized.");
    }

    void vel_factor_callback(std_msgs::Float32::Ptr msg)
    {
        vel_factor = msg->data;
    }
    void is_adjusting_callback(std_msgs::Bool::Ptr msg)
    {
        is_adjusting = msg->data;
    }

    void cmd_vel111_callback(geometry_msgs::Twist::Ptr msg)
    {
        twist.linear.x = msg->linear.x * vel_factor;
        twist.linear.y = msg->linear.y * vel_factor;
        if (is_adjusting ||
            (target_pose.theta == 0 && target_pose.x == 0 && target_pose.y == 0)) // 表示move_base导航结束
        {
            twist.angular.z = msg->angular.z;
            pub_cmd_vel.publish(twist);
        }
        else
        { // 由Cmd_vel_relay来控制实际旋转，这里注意一下，使用的时候尽可能保持yaw_tolerance<=局部规划器，不然可能会有bug
            try
            {
                listener_.lookupTransform("map", "base_link", ros::Time(0), map_base_transform);
                tf::Quaternion q = map_base_transform.getRotation();
                double roll, pitch, yaw;
                tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
                now_pose.x = map_base_transform.getOrigin().x();
                now_pose.y = map_base_transform.getOrigin().y();
                now_pose.theta = yaw;
                // ROS_INFO("x: %f, y: %f, yaw: %f", now_pose.x, now_pose.y, now_pose.theta);
                double delta_yaw = target_pose.theta - now_pose.theta;
                normalizeAngle(delta_yaw);
                if (fabs(delta_yaw) > yaw_tolerance)
                    twist.angular.z = delta_yaw * base_angular;
                else
                    twist.angular.z = 0;
                pub_cmd_vel.publish(twist);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("Failed to listen TF: %s", ex.what());
                twist.linear.x = 0;
                twist.linear.y = 0;
                twist.angular.z = 0;
            }
        }
    }
    void target_pose_callback(geometry_msgs::Pose2D::Ptr msg)
    {
        target_pose = *msg;
    }
    void timerCallback(const ros::TimerEvent &)
    {
        if (twist.linear.x == 0 && twist.linear.y == 0 && twist.angular.z == 0 && zero_times < 100)
        {
            pub_cmd_vel.publish(twist);
            zero_times++;
        }
        else if (twist.linear.x != 0 || twist.linear.y != 0 || twist.angular.z != 0)
        {
            zero_times = 0;
        }
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
    ros::init(argc, argv, "cmd_vel_ralay_node");
    ros::NodeHandle nh;

    Cmd_vel_ralay cmd_vel_ralay_node(&nh);

    ros::spin();

    return 0;
}

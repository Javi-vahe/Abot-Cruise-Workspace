#include "pid_follow_planner/pid_position_follow.h"

RobotCtrl::RobotCtrl()
{
    nh = ros::NodeHandle("~");
    nh.param<double>("max_x_speed", max_x_speed_, 1.0);
    nh.param<double>("max_y_speed", max_y_speed_, 1.0);

    nh.param<double>("p_value", p_value_, 0.5);
    nh.param<double>("i_value", i_value_, 1);
    nh.param<double>("d_value", d_value_, 1);

    nh.param<int>("plan_frequency", plan_freq_, 20);
    nh.param<double>("goal_dist_tolerance", goal_dist_tolerance_, 0.1);
    nh.param<double>("prune_ahead_distance", prune_ahead_dist_, 0.5);
    nh.param<std::string>("global_frame", global_frame_, "map");

    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel_origin", 10);
    local_path_pub_ = nh.advertise<nav_msgs::Path>("path", 5);
    end_move_base_pub_ = nh.advertise<std_msgs::Empty>("end_move_base", 1);
    global_path_sub_ = nh.subscribe("/move_base/GlobalPlanner/plan", 5, &RobotCtrl::GlobalPathCallback, this);
    target_pose_sub_ = nh.subscribe("/target_pose", 2, &RobotCtrl::target_pose_Callback, this);
    sub_is_adjusting = nh.subscribe("/is_adjusting", 1, &RobotCtrl::is_adjusting_callback, this);

    tf_listener_ = std::make_shared<tf::TransformListener>();

    plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_freq_), &RobotCtrl::Plan, this);
}

void RobotCtrl::target_pose_Callback(geometry_msgs::Pose2D::Ptr msg)
{
    if (msg->x == 0 && msg->y == 0 && msg->theta == 0)
    {
        plan_ = false;
        move_base_activing = false;
        ROS_INFO("move_base end, end planning!");
    }
}

void RobotCtrl::Plan(const ros::TimerEvent &event)
{
    nh.getParam("max_x_speed", max_x_speed_);
    nh.getParam("max_y_speed", max_y_speed_);
    nh.getParam("p_value", p_value_);
    if (plan_)
    {
        auto begin = std::chrono::steady_clock::now();
        auto start = ros::Time::now();
        UpdateTransform(tf_listener_, global_frame_,
                        global_path_.header.frame_id, global_path_.header.stamp,
                        global2path_transform_);
        std::cout << ros::Time::now() - start << std::endl;

        geometry_msgs::PoseStamped robot_pose;
        GetGlobalRobotPose(tf_listener_, global_path_.header.frame_id, robot_pose);

        if (GetEuclideanDistance(robot_pose, global_path_.poses.back()) <= goal_dist_tolerance_ || prune_index_ == global_path_.poses.size() - 1)
        {
            plan_ = false;
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = 0;
            cmd_vel_pub_.publish(cmd_vel);
            std_msgs::Empty end_msg;
            end_move_base_pub_.publish(end_msg);
            ROS_INFO("Planning Success!");
            return;
        }

        FindNearstPose(robot_pose, global_path_, prune_index_, prune_ahead_dist_);

        nav_msgs::Path prune_path, local_path;

        local_path.header.frame_id = global_frame_;
        prune_path.header.frame_id = global_frame_;

        geometry_msgs::PoseStamped tmp_pose;
        tmp_pose.header.frame_id = global_frame_;

        TransformPose(global2path_transform_, robot_pose, tmp_pose);
        prune_path.poses.push_back(tmp_pose);

        int i = prune_index_;

        while (i < global_path_.poses.size() && i - prune_index_ < 20)
        {

            TransformPose(global2path_transform_, global_path_.poses[i], tmp_pose);
            prune_path.poses.push_back(tmp_pose);
            i++;
        }
        // for (int i = prune_index_; i < global_path_.poses.size(); i++){
        //     TransformPose(global2path_transform_, global_path_.poses[i], tmp_pose);
        //     prune_path.poses.push_back(tmp_pose);

        // }

        GenTraj(prune_path, local_path);
        local_path_pub_.publish(local_path);

        geometry_msgs::Twist cmd_vel;
        FollowTraj(robot_pose, local_path, cmd_vel);
        cmd_vel_pub_.publish(cmd_vel);

        auto plan_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - begin);
        ROS_INFO("Planning takes %f ms and passed %d/%d.",
                 plan_time.count() / 1000.,
                 prune_index_,
                 static_cast<int>(global_path_.poses.size()));
    }
    else // plan_ == false
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;
        if (!is_adjusting)
            cmd_vel_pub_.publish(cmd_vel);
    }
}
void RobotCtrl::is_adjusting_callback(std_msgs::Bool::Ptr msg)
{
    is_adjusting = msg->data;
}

void RobotCtrl::FindNearstPose(geometry_msgs::PoseStamped &robot_pose, nav_msgs::Path &path, int &prune_index, double prune_ahead_dist)
{
    double dist_threshold = 10; 
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist;
    if (prune_index != 0)
    {
        sq_dist = GetEuclideanDistance(robot_pose, path.poses[prune_index - 1]);
    }
    else
    {
        sq_dist = 1e10;
    }

    double new_sq_dist = 0;
    while (prune_index < (int)path.poses.size())
    {
        new_sq_dist = GetEuclideanDistance(robot_pose, path.poses[prune_index]);
        if (new_sq_dist > sq_dist && sq_dist < sq_dist_threshold)
        {

            if ((path.poses[prune_index].pose.position.x - robot_pose.pose.position.x) *
                            (path.poses[prune_index - 1].pose.position.x - robot_pose.pose.position.x) +
                        (path.poses[prune_index].pose.position.y - robot_pose.pose.position.y) *
                            (path.poses[prune_index - 1].pose.position.y - robot_pose.pose.position.y) >
                    0 &&
                sq_dist > prune_ahead_dist)
            {
                prune_index--;
            }
            else
            {
                sq_dist = new_sq_dist;
            }

            break;
        }
        sq_dist = new_sq_dist;
        ++prune_index;
    }

    prune_index = std::min(prune_index, (int)(path.poses.size() - 1));
}

void RobotCtrl::FollowTraj(const geometry_msgs::PoseStamped &robot_pose,
                           const nav_msgs::Path &traj,
                           geometry_msgs::Twist &cmd_vel)
{

    geometry_msgs::PoseStamped robot_pose_1;
    GetGlobalRobotPose(tf_listener_, global_path_.header.frame_id, robot_pose_1);

    yaw_ = tf::getYaw(robot_pose_1.pose.orientation);

    // double diff_yaw = GetYawFromOrientation(traj.poses[0].pose.orientation)- GetYawFromOrientation(robot_pose.pose.orientation);
    double diff_yaw = atan2((traj.poses[1].pose.position.y - robot_pose.pose.position.y), (traj.poses[1].pose.position.x - robot_pose.pose.position.x));

    double diff_distance = GetEuclideanDistance(robot_pose, traj.poses[1]);

    if (diff_yaw > M_PI)
    {
        diff_yaw -= 2 * M_PI;
    }
    else if (diff_yaw < -M_PI)
    {
        diff_yaw += 2 * M_PI;
    }

    printf("diff_yaw: %f\n", diff_yaw);
    printf("diff_distance: %f\n", diff_distance);

    double vx_global = max_x_speed_ * cos(diff_yaw) * p_value_; //*diff_distance*p_value_;
    double vy_global = max_y_speed_ * sin(diff_yaw) * p_value_; //*diff_distance*p_value_;
    std::cout << "yaw_:" << yaw_ << std::endl;
    std::cout << "vx_gl:  " << vx_global << "   vy_gl:   " << vy_global << std::endl;

    cmd_vel.linear.x = vx_global * cos(yaw_) + vy_global * sin(yaw_);
    cmd_vel.linear.y = -vx_global * sin(yaw_) + vy_global * cos(yaw_);
    // ROS_INFO("vx:%f \t vy:%f", cmd_vel.linear.x, cmd_vel.linear.y);
    cmd_vel.angular.z = 0;

    float dist_to_goal = GetEuclideanDistance(robot_pose, global_path_.poses.back());
    if (dist_to_goal <= 1)
    {
        cmd_vel.linear.x *= dist_to_goal;
        cmd_vel.linear.y *= dist_to_goal;
    }
}

void RobotCtrl::GlobalPathCallback(const nav_msgs::PathConstPtr &msg)
{
    if (!msg->poses.empty())
    {
        global_path_ = *msg;
        prune_index_ = 0;
        plan_ = true;
        move_base_activing = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_position_follow");
    RobotCtrl robotctrl;
    ros::spin();
    return 0;
}


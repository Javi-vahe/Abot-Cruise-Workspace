#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include "robot_slam/Block.h"

class Block_judge
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_clicked_point;
    ros::Publisher pub_enable_identify;
    ros::ServiceServer server_;
    ros::Timer timer_;

    geometry_msgs::Pose2D mid_point, now_pose;
    tf::TransformListener listener_;
    tf::StampedTransform map_base_transform;

    int now_block = 1;
    int next_block = 2;
    std::vector<int> blocks = {1, 2, 3, 4};
    bool listen_tf = false;

public:
    Block_judge()
    {
        nh_ = ros::NodeHandle("~");
        double mid_point_x, mid_point_y;
        nh_.param("mid_point_x", mid_point_x, -1.53606);
        nh_.param("mid_point_y", mid_point_y, 1.62314);
        std::cout << "mid_point_x" << mid_point_x << std::endl;
        std::cout << "mid_point_y" << mid_point_y << std::endl;

        sub_clicked_point = nh_.subscribe("/clicked_point", 1, &Block_judge::clicked_point_cb, this);
        pub_enable_identify = nh_.advertise<std_msgs::Empty>("/enable_identify", 10);
        server_ = nh_.advertiseService("/block_judge", &Block_judge::service_cb, this);
        timer_ = nh_.createTimer(ros::Duration(0.5), &Block_judge::timerCallback, this);

        mid_point.x = mid_point_x;
        mid_point.y = mid_point_y;
    }

    void timerCallback(const ros::TimerEvent &)
    {
        if (!listen_tf)
            return;
        // else
        update_block_num();
        if (now_block == next_block)
        {
            listen_tf = false;
            std_msgs::Empty msg;
            pub_enable_identify.publish(msg);
        }
    }

    bool service_cb(robot_slam::Block::Request &req, robot_slam::Block::Response &res)
    {
        req;
        // update_block_num();
        res.next_block = blocks.at(0);
        next_block = res.next_block;
        // 使用 std::remove 将需要删除的元素移动到 vector 的末尾
        blocks.erase(std::remove(blocks.begin(), blocks.end(), blocks.at(0)), blocks.end());
        listen_tf = true;
        return true;
    }

    void update_block_num()
    {
        try
        {
            listener_.lookupTransform("map", "base_link", ros::Time(0), map_base_transform);
            now_pose.x = map_base_transform.getOrigin().x();
            now_pose.y = map_base_transform.getOrigin().y();
            if (now_pose.x > mid_point.x)
            {
                if (now_pose.y > mid_point.y)
                    now_block = 4;
                else
                    now_block = 1;
            }
            else
            {
                if (now_pose.y > mid_point.y)
                    now_block = 3;
                else
                    now_block = 2;
            }
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("Failed to listen TF: %s", ex.what());
        }
    }

    void clicked_point_cb(geometry_msgs::PointStamped::Ptr msg)
    {
        update_block_num(msg->point.x, msg->point.y);
        std::cout << "point.x=" << msg->point.x << "\tpoint.y=" << msg->point.y << std::endl;
        std::cout << "block num: " << now_block << std::endl;
        // blocks.erase(std::remove(blocks.begin(), blocks.end(), now_block), blocks.end());
        // blocks.erase(std::remove(blocks.begin(), blocks.end(), blocks.at(0)), blocks.end());
        // for (int num : blocks)
        // {
        //     std::cout << num << " ";
        // }
        // std::cout << std::endl;
    }
    void update_block_num(double &x, double &y)
    {
        if (x > mid_point.x)
        {
            if (y > mid_point.y)
                now_block = 4;
            else
                now_block = 1;
        }
        else
        {
            if (y > mid_point.y)
                now_block = 3;
            else
                now_block = 2;
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "block_judge");
    Block_judge node;
    ros::spin();
    return 0;
}
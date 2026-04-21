#include "abot_imu/abot_imu.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pibot_imu");
  ros::NodeHandle nh, pnh("~");
  AbotIMU abot_imu(nh, pnh);

  ros::spin();

  return 0;
}

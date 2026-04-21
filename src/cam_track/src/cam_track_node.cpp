#include <ros/ros.h>
#include <cam_track/cam_track_node.hpp>

using namespace cam_track;
int main(int argc,char** argv){
    ros::init(argc,argv,"cam_track_node");
    ros::NodeHandle nh("~");
    CamTrack cam_tracker(nh);
    cam_tracker.initROSModule();
    ros::spin();
    return 0;

}

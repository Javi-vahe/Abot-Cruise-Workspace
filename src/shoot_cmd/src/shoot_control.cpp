#include "../include/shoot_cmd/shoot_control.h"

Shoot_Control::Shoot_Control(){
}

Shoot_Control::~Shoot_Control(){
    s.close();
}

bool Shoot_Control::init(const char *port_name,int baudrate){
    return s.open( port_name, baudrate, 'N', 8, 1);
} 

void Shoot_Control::mSleep(int mtime)
{
    struct timeval tv;
    unsigned long usec = static_cast<unsigned long>(mtime*1000) ;
    tv.tv_sec = usec / 1000000;
    tv.tv_usec = usec % 1000000;

    int err;
    do {
        err = select(0, NULL, NULL, NULL, &tv);
    } while(err < 0 && errno == EINTR);
}

void Shoot_Control::uSleep(int usec)
{
    struct timeval tv;
    tv.tv_sec = usec / 1000000;
    tv.tv_usec = usec % 1000000;

    int err;
    do {
        err = select(0, NULL, NULL, NULL, &tv);
    } while(err < 0 && errno == EINTR);
}

void Shoot_Control::shoot()
{
    char buf[8] = {0x55, 0x01, 0x12, 0x00, 0x00, 0x00, 0x01, 0x69};
    s.send(buf, 8);
}

void Shoot_Control::stop_shoot()
{
    char buf[8] = {0x55, 0x01, 0x11, 0x00, 0x00, 0x00, 0x01, 0x68};
    s.send(buf, 8);
}

Shoot_Control sc;

void shoot_cb(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data == "shoot")
    {
        sc.shoot();
    }
    else if (msg->data == "stopshoot")
    {
        sc.stop_shoot();
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "shoot_control");
    ros::NodeHandle n;
    ros::Subscriber shoot_sub = n.subscribe("/shoot", 1000, shoot_cb);;
    char portname[20];
    sprintf(portname,"/dev/shoot");
    sc.init(portname, 9600);
    ros::Rate loop_rate(200);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
}


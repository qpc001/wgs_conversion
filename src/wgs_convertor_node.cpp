#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <XmlRpcException.h>

#include "wgs_conversions.h"
#include "wgs_convertor.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, " ");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ROS_INFO("Start");

    wgs_convertor convertor(nh,nh_priv);

    ros::spin();
    return 0;
}

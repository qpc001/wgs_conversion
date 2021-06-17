#ifndef SWRI_INS_TRANSFORM_H
#define SWRI_INS_TRANSFORM_H

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "geometry_msgs/TransformStamped.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <XmlRpcException.h>

#include <tf/transform_datatypes.h>
#include <swri_math_util/constants.h>
#include <swri_math_util/trig_util.h>
#include <swri_transform_util/earth_constants.h>
#include <swri_transform_util/transform.h>
#include <swri_transform_util/transform_manager.h>
#include <swri_yaml_util/yaml_util.h>
#include <swri_transform_util/frames.h>

#include "tf_listener.hpp"

class swri_ins_transform
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    swri_ins_transform(ros::NodeHandle nh, ros::NodeHandle nh_priv);
    void gpsFixCallback(sensor_msgs::NavSatFixConstPtr);
    void insCallback(nav_msgs::OdometryConstPtr);
    void velCallback(geometry_msgs::TwistStampedConstPtr);
    bool InitCalibration();

private:
    //! @brief GPS subscriber
    ros::Subscriber gps_sub_;

    //! @brief GPS/IMU Fuse Data subscriber
    ros::Subscriber ins_sub_;

    //! @brief Velocity subscriber (this for waypoint saver)
    ros::Subscriber vel_sub_;

    //! @brief Publisher for gps/fix data (type navsat_msgs)
    ros::Publisher gps_pos_in_map_pub_;

    //! @brief Publisher for ins data[in UTM/ENU Frame] (type odomerty)
    ros::Publisher ins_pos_in_map_pub_;

    //! @brief Publisher for ins position data (topic type geometry_msgs/PoseStamped)
    ros::Publisher ins_pos_waypoint_pub_;

    //! @brief Publisher for ins velocity data (topic type geometry_msgs/TwistStamped)
    ros::Publisher ins_vel_waypoint_pub_;

    std::shared_ptr<TFListener> base_to_imu_ptr_;                      // Base到IMU的TF变换
    Eigen::Matrix4f base_to_imu_ = Eigen::Matrix4f::Identity();        // Base到IMU的变换

    Eigen::Vector3d linear_velocity_=Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity_=Eigen::Vector3d::Zero();


    /// buffer for tf lookups not related to fixed-frame
    boost::shared_ptr<tf::TransformListener> tf_;
    swri_transform_util::TransformManagerPtr tf_manager_;
    swri_transform_util::Transform transform_;
    std::string target_frame_;
    std::string source_frame_= swri_transform_util::_wgs84_frame;
};

#endif // SWRI_TRANSFORM_H

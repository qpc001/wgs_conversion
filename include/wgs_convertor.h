#ifndef WGS_CONVERTOR_H
#define WGS_CONVERTOR_H

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

#include "wgs_conversions.h"
#include "utm_conversion_utils.h"

enum CONVERT_MODE {UTM,ENU};
const int POSITION_SIZE = 3;
const int POSE_SIZE = 6;

class wgs_convertor
{
public:
    wgs_convertor(ros::NodeHandle nh, ros::NodeHandle nh_priv);

    //CALLBACK
    void gpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg);
    void insCallback(const nav_msgs::OdometryConstPtr& msg);
    void velCallback(const geometry_msgs::TwistStampedConstPtr &msg);
    void lla2enu(double lla[], double enu_output[]);
    void lla2utm(double lla[], double utm_output[]);

public:
    //! @brief 原点经纬高
    double latitude_ori=0;
    double longitude_ori=0;
    double height_ori=0;

    //! @brief 坐标系
    std::string imu_frame_id_;
    std::string gps_frame_id_;
    std::string base_link_frame_id_;

    std::string map_frame_id_;
    std::string odom_frame_id;

    //! @brief 消息时间戳
    ros::Time gps_update_time_;

    //! @brief 模式
    CONVERT_MODE mode_;

    //! @brief 使用M2融合输出
    bool use_M2_fuse_;

    //! @brief GPS subscriber
    ros::Subscriber gps_sub_;

    //! @brief GPS/IMU Fuse Data subscriber
    ros::Subscriber ins_sub_;

    //! @brief Velocity subscriber (this for waypoint saver)
    ros::Subscriber vel_sub_;

    //! @brief GPS更新标志位
    bool gps_updated_;

    WgsConversions converter;

    //! @brief Latest GPS data, stored as UTM coords
    //! gps 到 utm或者ENU的变换
    tf2::Transform latest_output_transform_;

    //! @brief Covariance for most recent GPS/UTM data
    Eigen::MatrixXd latest_output_covariance_;

    //! @brief timer calling periodicUpdate
    ros::Timer processTimer_;

private:
  //! @brief callback function which is called for periodic updates
  void process(const ros::TimerEvent& event);

  nav_msgs::Odometry prepareGpsOdometry(ros::Time time);

  void get_base_link_inWorld(const tf2::Transform &gps_output_transform_,
                               tf2::Transform &base_link_output_transform,
                               const ros::Time &transform_time);

  //! @brief Publisher for gps/fix data (type navsat_msgs)
  ros::Publisher gps_pos_in_map_pub_;

  //! @brief Publisher for ins data[in UTM/ENU Frame] (type odomerty)
  ros::Publisher ins_pos_in_map_pub_;

  //! @brief Publisher for ins position data (topic type geometry_msgs/PoseStamped)
  ros::Publisher ins_pos_waypoint_pub_;

  //! @brief Publisher for ins velocity data (topic type geometry_msgs/TwistStamped)
  ros::Publisher ins_vel_waypoint_pub_;

  //! @brief Transform buffer for managing coordinate transforms
  tf2_ros::Buffer tf_buffer_;

  //! @brief Transform listener for receiving transforms
  tf2_ros::TransformListener tf_listener_;

  //! @brief TF broadcaster
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  //! @brief Parameter that specifies the how long we wait for a transform to become available.
  ros::Duration transform_timeout_;
};

#endif // WGS_CONVERTOR_H

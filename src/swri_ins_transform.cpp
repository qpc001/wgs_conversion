#include "swri_ins_transform.h"

swri_ins_transform::swri_ins_transform(ros::NodeHandle nh, ros::NodeHandle nh_priv)
{
    // "gps/fix" 来自GPS/RTK FIX数据
    gps_sub_ = nh.subscribe("gps/fix", 10, &swri_ins_transform::gpsFixCallback,this);
    // "gps/ins" 组合导航数据
    ins_sub_ = nh.subscribe("gps/ins",100, &swri_ins_transform::insCallback,this);
    // "vel"
    vel_sub_ = nh.subscribe("vel",100,&swri_ins_transform::velCallback,this);

    // 发布器：将GPS(经纬度) 转换到 地图坐标系（x，y）之后的数据
    gps_pos_in_map_pub_ = nh.advertise<nav_msgs::Odometry>("localization/pure_rtk", 10);

    // 发布器：将组合导航融合之后的(经纬度) 转换到 UTM/ENU之后的数据
    ins_pos_in_map_pub_ = nh.advertise<nav_msgs::Odometry>("localization/rtk_ins",100);

    base_to_imu_ptr_ = std::make_shared<TFListener>(nh, "/imu_link", "/base_link");

    // qpc modify: 2021-05-01
    tf_ = boost::make_shared<tf::TransformListener>();
    tf_manager_ = boost::make_shared<swri_transform_util::TransformManager>();
    tf_manager_->Initialize(tf_);
    target_frame_="map";
}

void swri_ins_transform::gpsFixCallback(sensor_msgs::NavSatFixConstPtr msg){
    if(!InitCalibration()){
        return;
    }
    if (tf_manager_->LocalXyUtil()->Initialized())
    {
        double x;
        double y;
        tf_manager_->LocalXyUtil()->ToLocalXy(msg->latitude, msg->longitude, x, y);
        //std::cout<<"Local X:"<<x<<" y:"<<y<<std::endl;
    }
}

void swri_ins_transform::insCallback(nav_msgs::OdometryConstPtr msg)
{
    if(!InitCalibration()){
        return;
    }
    if (tf_manager_->LocalXyUtil()->Initialized())
    {
        double x;
        double y;
        tf_manager_->LocalXyUtil()->ToLocalXy(msg->pose.pose.position.y, msg->pose.pose.position.x, x, y);
        std::cout<<"Local ins X:"<<x<<" y:"<<y<<std::endl;
        double ref_altitude = tf_manager_->LocalXyUtil()->ReferenceAltitude();
        double z = msg->pose.pose.position.z - ref_altitude;

        nav_msgs::Odometry rtk_ins;
        rtk_ins.header=msg->header;

        Eigen::Vector3f trans(x,y,z);
        Eigen::Quaternionf orientation;
        orientation.w()=msg->pose.pose.orientation.w;
        orientation.x()=msg->pose.pose.orientation.x;
        orientation.y()=msg->pose.pose.orientation.y;
        orientation.z()=msg->pose.pose.orientation.z;

        Eigen::Matrix4f pose=Eigen::Matrix4f::Identity();
        pose.block(0,0,3,3)=orientation.matrix();
        pose.block(0,3,3,1)=trans;

        pose = pose * base_to_imu_;
        trans=pose.block(0,3,3,1);

        rtk_ins.pose.pose.position.x=trans.x();
        rtk_ins.pose.pose.position.y=trans.y();
        rtk_ins.pose.pose.position.z=trans.z();

        Eigen::Matrix3f R=Eigen::Matrix3f::Identity();
        R=pose.block(0,0,3,3);
        Eigen::Quaternionf q(R);

        rtk_ins.pose.pose.orientation.w=q.w();
        rtk_ins.pose.pose.orientation.x=q.x();
        rtk_ins.pose.pose.orientation.y=q.y();
        rtk_ins.pose.pose.orientation.z=q.z();

        auto linear_velocity = R.inverse().cast<double>()*linear_velocity_;
        auto angular_velocity= base_to_imu_.block(0,0,3,3).transpose().cast<double>()*angular_velocity_;
        rtk_ins.twist.twist.linear.x=linear_velocity.x();
        rtk_ins.twist.twist.linear.y=linear_velocity.y();
        rtk_ins.twist.twist.linear.z=linear_velocity.z();
        rtk_ins.twist.twist.angular.x=angular_velocity.x();
        rtk_ins.twist.twist.angular.y=angular_velocity.y();
        rtk_ins.twist.twist.angular.z=angular_velocity.z();

        rtk_ins.pose.covariance=msg->pose.covariance;
        ins_pos_in_map_pub_.publish(rtk_ins);
    }
}

void swri_ins_transform::velCallback(geometry_msgs::TwistStampedConstPtr vel_msg)
{
    // the topic [/vel] about velocity is  [v_n , v_e , v_d ]
    // So, change the order to  [v_e, v_n, v_d]
    Eigen::Vector3d linear_velocity(0,0,0);
    linear_velocity.x()=vel_msg->twist.linear.y;
    linear_velocity.y()=vel_msg->twist.linear.x;
    linear_velocity.z()=vel_msg->twist.linear.z;
    Eigen::Vector3d angular_velocity(0,0,0);
    angular_velocity.x()=vel_msg->twist.angular.x;
    angular_velocity.y()=vel_msg->twist.angular.y;
    angular_velocity.z()=vel_msg->twist.angular.z;

    linear_velocity_=linear_velocity;
    angular_velocity_=angular_velocity;
}

// 读取外参
bool swri_ins_transform::InitCalibration() {
    static bool calibration_received = false;
    static bool lidar_to_imu_flag=false;
    static bool base_to_imu_flag=false;
    if (!calibration_received){
        if (!base_to_imu_flag){
            if (base_to_imu_ptr_->LookupData(base_to_imu_)){
                base_to_imu_flag = true;
            }
        }
        if (base_to_imu_flag){
            calibration_received = true;
        }
    }
    std::cout<<"Read baselink to imu param finished."<<std::endl;
    return calibration_received;
}

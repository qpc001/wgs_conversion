#include "wgs_convertor.h"

//utils
bool lookupTransformSafe(const tf2_ros::Buffer &buffer,
                         const std::string &targetFrame,
                         const std::string &sourceFrame,
                         const ros::Time &time,
                         const ros::Duration &timeout,
                         tf2::Transform &targetFrameTrans,
                         const bool silent)
{
    bool retVal = true;

    // First try to transform the data at the requested time
    try
    {
        tf2::fromMsg(buffer.lookupTransform(targetFrame, sourceFrame, time, timeout).transform,
                     targetFrameTrans);
    }
    catch (tf2::TransformException &ex)
    {
        // The issue might be that the transforms that are available are not close
        // enough temporally to be used. In that case, just use the latest available
        // transform and warn the user.
        try
        {
            tf2::fromMsg(buffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0)).transform,
                         targetFrameTrans);

            if (!silent)
            {
                ROS_WARN_STREAM_THROTTLE(2.0, "Transform from " << sourceFrame << " to " << targetFrame <<
                                         " was unavailable for the time requested. Using latest instead.\n");
            }
        }
        catch(tf2::TransformException &ex)
        {
            if (!silent)
            {
                ROS_WARN_STREAM_THROTTLE(2.0, "Could not obtain transform from " << sourceFrame <<
                                         " to " << targetFrame << ". Error was " << ex.what() << "\n");
            }

            retVal = false;
        }
    }

    // Transforming from a frame id to itself can fail when the tf tree isn't
    // being broadcast (e.g., for some bag files). This is the only failure that
    // would throw an exception, so check for this situation before giving up.
    if (!retVal)
    {
        if (targetFrame == sourceFrame)
        {
            targetFrameTrans.setIdentity();
            retVal = true;
        }
    }

    return retVal;
}

//=========================================================================

wgs_convertor::wgs_convertor(ros::NodeHandle nh, ros::NodeHandle nh_priv)
    : tf_listener_(tf_buffer_),
      transform_timeout_(ros::Duration(0.0)),
      gps_updated_(false),
      base_link_frame_id_("base_link"),
      map_frame_id_("map"),
      gps_frame_id_("gps"),
      imu_frame_id_("imu_link"),
      use_M2_fuse_(true)
{
    latest_output_covariance_.resize(POSE_SIZE, POSE_SIZE);

    // 模式选择（UTM，ENU）
    std::string mode;
    nh_priv.param("CONVERT_MODE",mode,std::string("ENU"));
    if(mode=="ENU"){
        mode_=CONVERT_MODE::ENU;
    }else{
        mode_=CONVERT_MODE::UTM;
    }

    // 读取手动设置的地图原点的gps数据
    if (nh.hasParam("datum"))
    {

        std::vector<double> ori_vec(3,0);
        nh.param<std::vector<double>>("datum", ori_vec, std::vector<double>());

        latitude_ori=ori_vec[0];
        longitude_ori=ori_vec[1];
        altitude_ori=ori_vec[2];

        ROS_INFO("datum: [%.12f  %.12f  %.12f]",latitude_ori,longitude_ori,altitude_ori);
    }

    // "gps/fix" 来自GPS/RTK FIX数据
    gps_sub_ = nh.subscribe("gps/fix", 1, &wgs_convertor::gpsFixCallback,this);
    // "gps/ins" 组合导航数据
    ins_sub_ = nh.subscribe("gps/ins",50, &wgs_convertor::insCallback,this);

    // 发布器：将GPS(经纬度) 转换到 地图坐标系（x，y）之后的数据
    gps_pos_in_map_pub_ = nh.advertise<nav_msgs::Odometry>("odometry/gpsFix", 10);

    // 发布器：将组合导航融合之后的(经纬度) 转换到 UTM/ENU之后的数据
    ins_pos_in_map_pub_ = nh.advertise<nav_msgs::Odometry>("odometry/ins",50);

    //! 下面的是给waypoint saver 专门做的
    // 1. 订阅器：订阅速度
    vel_sub_ = nh.subscribe("/vel",50,&wgs_convertor::velCallback,this);
    // 2. 发布器：将坐标和速度信息发布，用来记录waypoint ====> [暂时策略]
    ins_pos_waypoint_pub_=nh.advertise<geometry_msgs::PoseStamped>("/current_pose",50);
    ins_vel_waypoint_pub_=nh.advertise<geometry_msgs::TwistStamped>("/current_velocity",50);
}


// RTK_Fix数据回调
void wgs_convertor::gpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg)
{
    // 取msg的坐标系，作为 gps坐标系
    gps_frame_id_ = msg->header.frame_id;

    // ROS_INFO("got gps msg");

    if (gps_frame_id_.empty())
    {
        ROS_WARN_STREAM_ONCE("NavSatFix message has empty frame_id. Will assume navsat device is mounted at robot's "
                             "origin.");
    }

    // Make sure the GPS data is usable
    // 确保gps数据可用
    bool good_gps = (msg->status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX &&
            !std::isnan(msg->altitude) &&
            !std::isnan(msg->latitude) &&
            !std::isnan(msg->longitude));

    double output[3]={0,0,0};
    if (good_gps)
    {
        double lla[]={msg->latitude,msg->longitude,msg->altitude};

        if(mode_==CONVERT_MODE::ENU){
            lla2enu(lla,output);
            ROS_INFO("ENU Coordinate:[%f  %f  %f]",output[0],output[1],output[2]);
        }else if(mode_==CONVERT_MODE::UTM){
            lla2utm(lla,output);
            ROS_INFO("UTM Coordinate:[%f  %f  %f]",output[0],output[1],output[2]);
        }

        // 转成tf2::Transform
        latest_output_transform_.setOrigin(tf2::Vector3(output[0], output[1], output[2]));
        latest_output_covariance_.setZero();

        // 复制协方差
        // Copy the measurement's covariance matrix so that we can rotate it later
        for (size_t i = 0; i < POSITION_SIZE; i++)
        {
            for (size_t j = 0; j < POSITION_SIZE; j++)
            {
                latest_output_covariance_(i, j) = msg->position_covariance[POSITION_SIZE * i + j];
            }
        }

        // 更新时间戳
        gps_update_time_ = msg->header.stamp;
        gps_updated_ = true;


        ROS_INFO("GPS-RTK in map :[%f  %f  %f]",
                 latest_output_transform_.getOrigin().x(),
                 latest_output_transform_.getOrigin().y(),
                 latest_output_transform_.getOrigin().z());

        nav_msgs::Odometry gps_pose;
        tf2::toMsg(latest_output_transform_, gps_pose.pose.pose);
        gps_pose.header.stamp=msg->header.stamp;;
        gps_pose.header.frame_id=map_frame_id_;
        gps_pose.child_frame_id=base_link_frame_id_; //base_link_frame_id_;
        gps_pose.pose.pose.position.z = ((mode_==CONVERT_MODE::UTM) ? 0.0 : gps_pose.pose.pose.position.z);

        //gps_pose.pose.pose.position.z = 0.1;

        // Copy the measurement's covariance matrix so that we can rotate it later
        // 复制协方差
        for (size_t i = 0; i < POSE_SIZE; i++)
        {
            for (size_t j = 0; j < POSE_SIZE; j++)
            {
                gps_pose.pose.covariance[POSE_SIZE * i + j] = latest_output_covariance_(i, j);
            }
        }

        // 发布nav::odom消息
        gps_pos_in_map_pub_.publish(gps_pose);
    }
}

// ZXY顺序欧拉角转四元数
template <typename T>
Eigen::Quaternion<T> ZXYeulerToQuaternion(T roll, T pitch ,T yaw)
{
    T coeff = static_cast<T>(0.5);
    T r = roll * coeff;
    T p = pitch * coeff;
    T y = yaw * coeff;

    T sr = std::sin(r);
    T sp = std::sin(p);
    T sy = std::sin(y);

    T cr = std::cos(r);
    T cp = std::cos(p);
    T cy = std::cos(y);

    T qw = cr * cp * cy - sr * sp * sy;
    T qx = cr * sp * cy - sr * cp * sy;
    T qy = cr * sp * sy + sr * cp * cy;
    T qz = cr * cp * sy + sr * sp * cy;

    if (qw < 0.0) {
        return {-qw, -qx, -qy, -qz};
    }
    return {qw, qx, qy, qz};
}

void wgs_convertor::insCallback(const nav_msgs::OdometryConstPtr &msg)
{
    //! @brief 这里将经纬度转换成UTM或ENU，
    //! @brief 然后，将IMU的数据转换成base_link的数据（主要是姿态）

    // 取保gps数据可用
    bool good_gps = (
                !std::isnan(msg->pose.pose.position.x) &&       // 经度
                !std::isnan(msg->pose.pose.position.y) &&       // 纬度
                !std::isnan(msg->pose.pose.position.z));        // 高度
    if (good_gps)
    {
        double lla[]={msg->pose.pose.position.y,
                      msg->pose.pose.position.x,
                      msg->pose.pose.position.z};

        double output[3]={0,0,0};

        if(mode_==CONVERT_MODE::ENU){
            lla2enu(lla,output);
            //ROS_INFO("ENU Coordinate:[%f  %f  %f]",output[0],output[1],output[2]);
        }else if(mode_==CONVERT_MODE::UTM){
            lla2utm(lla,output);
            //ROS_INFO("UTM Coordinate:[%f  %f  %f]",output[0],output[1],output[2]);
        }

        // 这里仍然是IMU在map坐标系的坐标
        nav_msgs::Odometry ins_odom_convert_result=*msg;
        ins_odom_convert_result.pose.pose.position.x=output[0];
        ins_odom_convert_result.pose.pose.position.y=output[1];
        ins_odom_convert_result.pose.pose.position.z=((mode_==CONVERT_MODE::UTM) ? 0.0 : msg->pose.pose.position.z);

        // 四元数 对应的欧拉角是 ZXY312顺序的
        tf2::Quaternion quat;
        tf2::fromMsg(ins_odom_convert_result.pose.pose.orientation,quat);

        // ROS定义 顺时针为正 , 而组合导航定义 逆时针为正
        // rpy 指的是 从 ENU坐标系到IMU坐标系 的旋转变换
        // 这里需要的是 IMU坐标系到ENU坐标系的变换，所以取符号
        //Eigen::Quaterniond q = ZXYeulerToQuaternion<double>(-roll,-pitch,-yaw);

        //  Eigen 转 geometry_msg
        //tf2::fromMsg(q,quat);
        //geometry_msgs::Quaternion q_msg = tf2::toMsg(q);
        //ins_odom_convert_result.pose.pose.orientation=q_msg;

        // 由于msg内的姿态是 IMU的姿态，现在要求车的姿态(base_link)
        tf2::Transform imu_2_bl_tf;
        bool can_transform = lookupTransformSafe(tf_buffer_,
                                                 base_link_frame_id_,            // base_link
                                                 imu_frame_id_,                  // imu_link
                                                 msg->header.stamp,
                                                 transform_timeout_,
                                                 imu_2_bl_tf,
                                                 false);

        if (can_transform){
            //tf2::Quaternion q_imu_2_bl = imu_2_bl_tf.getRotation();         // IMU坐标系到base_link的旋转变换

            // 取消息msg，构造 imu/gps坐标系到map坐标系的变换
            tf2::Transform imu_2_map;
            tf2::Vector3 v;
            tf2::fromMsg(ins_odom_convert_result.pose.pose.position,v);
            imu_2_map.setOrigin(v);
            tf2::Quaternion q;
            fromMsg(msg->pose.pose.orientation, q);
            imu_2_map.setRotation(q);
            //tf2::fromMsg(msg,imu_2_map);      //这句不能用，不知道为什么

            // 得到 base_link 坐标系 到 map 坐标系的变换
            tf2::Transform bl_2_map_tf = imu_2_map*imu_2_bl_tf.inverse();

            // 重新设置position和orientation， 这是base_link 坐标系 到 map 坐标系的变换
            ins_odom_convert_result.pose.pose.position.x = bl_2_map_tf.getOrigin().x();
            ins_odom_convert_result.pose.pose.position.y = bl_2_map_tf.getOrigin().y();
            ins_odom_convert_result.pose.pose.position.z = bl_2_map_tf.getOrigin().z();

            ins_odom_convert_result.pose.pose.orientation=tf2::toMsg(bl_2_map_tf.getRotation());

            //Debug
            //tf2Scalar yaw, pitch, roll;
            //tf2::Matrix3x3 mat(bl_2_map_tf.getRotation());
            //mat.getEulerYPR(yaw, pitch, roll);
            //ROS_INFO("Yaw [%f]",yaw);

            ins_odom_convert_result.header.frame_id=map_frame_id_;
            ins_odom_convert_result.child_frame_id=base_link_frame_id_;


            /// This if for waypoint saver:
            tf2::Stamped<tf2::Transform> current_pose_stamped(bl_2_map_tf,msg->header.stamp,map_frame_id_);
            geometry_msgs::PoseStamped current_pose_4_waypoint;
            tf2::toMsg(current_pose_stamped,current_pose_4_waypoint);

            geometry_msgs::TransformStamped transformTfGeom = tf2::toMsg(current_pose_stamped);
            transformTfGeom.header.frame_id=map_frame_id_;
            transformTfGeom.child_frame_id=base_link_frame_id_;

            ins_pos_in_map_pub_.publish(ins_odom_convert_result);
            //ins_pos_waypoint_pub_.publish(current_pose_4_waypoint);
            //tf_broadcaster_.sendTransform(transformTfGeom);
        }
        else
        {
            ROS_WARN_STREAM_THROTTLE(5.0, "Could not obtain " << imu_frame_id_ << "->" << base_link_frame_id_ <<
                                     " transform.");
        }
    }
}

void wgs_convertor::velCallback(const geometry_msgs::TwistStampedConstPtr &msg)
{
    geometry_msgs::TwistStamped vel_=*msg;
    vel_.twist.linear.x=sqrt(pow(vel_.twist.linear.x,2)+
                             pow(vel_.twist.linear.y,2));
    vel_.twist.linear.y=0;
    vel_.twist.linear.z=0;
    vel_.header.frame_id="base_link";
    //ins_vel_waypoint_pub_.publish(vel_);
}

//===================================== 两种坐标系转换 ======================================//
void wgs_convertor::lla2enu(double lla[], double enu_output[])
{
    double ref[]={latitude_ori,longitude_ori,altitude_ori};
    converter.lla2enu(enu_output,lla,ref);
}

void wgs_convertor::lla2utm(double lla[], double utm_output[])
{
    double utm_north=0;
    double utm_east=0;
    double utm_meridian_convergence=0;
    std::string UTMZone;
    NavsatConversions::LLtoUTM(lla[0],lla[1],utm_north,utm_east,UTMZone,utm_meridian_convergence);
    utm_meridian_convergence *=NavsatConversions::RADIANS_PER_DEGREE;

    double ref_north=0;
    double ref_east=0;
    double utm_meridian_convergence_ref=0;
    NavsatConversions::LLtoUTM(latitude_ori,longitude_ori,ref_north,ref_east,UTMZone,utm_meridian_convergence_ref);
    utm_meridian_convergence_ref *=NavsatConversions::RADIANS_PER_DEGREE;


    utm_output[1]=utm_north-ref_north;
    utm_output[0]=utm_east-ref_east;

    // offset
    // utm矫正
    double yaw_offset=utm_meridian_convergence;
    yaw_offset=utm_meridian_convergence_ref;
    tf2::Quaternion offset_quat;
    offset_quat.setRPY(0.0,0.0,-yaw_offset);
    tf2::Vector3 correct_utm=tf2::quatRotate(offset_quat,tf2::Vector3(utm_output[0],utm_output[1],0));

    utm_output[1]=correct_utm[1];
    utm_output[0]=correct_utm[0];
}


//void wgs_convertor::process(const ros::TimerEvent &event)
//{
//    // 处理，发布
//    if (gps_updated_)
//    {
//        // 本来是想将数据从gps变成base_link的，但是考虑了一下，还是直接取GPS坐标系的就可以了
//        nav_msgs::Odometry msg=prepareGpsOdometry(gps_update_time_);

//        // 发布nav::odom消息
//        gps_pos_in_map_pub_.publish(msg);

//        gps_updated_=false;
//    }
//}

//nav_msgs::Odometry wgs_convertor::prepareGpsOdometry(ros::Time time)
//{

//    //tf2::Transform base_link_output_transform;
//    //get_base_link_inWorldPose(latest_output_transform_, base_link_output_transform,time);

//    //
//    ROS_INFO("GPS in map :[%f  %f  %f]",
//             latest_output_transform_.getOrigin().x(),
//             latest_output_transform_.getOrigin().y(),
//             latest_output_transform_.getOrigin().z());

//    nav_msgs::Odometry gps_pose;
//    tf2::toMsg(latest_output_transform_, gps_pose.pose.pose);
//    gps_pose.header.stamp=time;
//    gps_pose.header.frame_id=map_frame_id_;
//    gps_pose.child_frame_id=base_link_frame_id_; //base_link_frame_id_;
//    gps_pose.pose.pose.position.z = ((mode_==CONVERT_MODE::UTM) ? 0.0 : gps_pose.pose.pose.position.z);

//    //gps_pose.pose.pose.position.z = 0.1;

//    // Copy the measurement's covariance matrix so that we can rotate it later
//    // 复制协方差
//    for (size_t i = 0; i < POSE_SIZE; i++)
//    {
//        for (size_t j = 0; j < POSE_SIZE; j++)
//        {
//            gps_pose.pose.covariance[POSE_SIZE * i + j] = latest_output_covariance_(i, j);
//        }
//    }

//    return gps_pose;
//}

///// 这里没用到
///// 由于base_link 到 utm或者ENU的旋转部分这里没法获得，还是直接返回GPS的 UTM或者ENU坐标好了
///// 将ENU或者UTM数据 转换为 以base_link的
//void wgs_convertor::get_base_link_inWorld(const tf2::Transform &gps_output_transform_,
//                                          tf2::Transform &base_link_output_transform,
//                                          const ros::Time &transform_time)
//{
//    ROS_INFO("Begin get_base_link_inWorldPose");
//    base_link_output_transform.setIdentity();

//    // Remove the offset from base_link
//    // gps_offset_rotated ： gps传感器坐标系 到 base_link 的变换
//    tf2::Transform gps_2_base_link;
//    bool can_transform = lookupTransformSafe(tf_buffer_,
//                                             base_link_frame_id_,
//                                             gps_frame_id_,
//                                             transform_time,
//                                             transform_timeout_,
//                                             gps_2_base_link,
//                                             false);

//    if (can_transform)
//    {
//        // gps_output_transform_： gps 到 utm或者ENU的变换
//        // gps_2_base_link: gps 到 base_link 的变换
//        ///base_link_output_transform： base_link 到 utm 或者 ENU的变换
//        ///[注意] 这里的tf变换顺序是从左往右看
//        base_link_output_transform =  gps_2_base_link.inverse()*gps_output_transform_;

//        // 或者？ gps_output_transform_ * gps_2_base_link.inverse()

//        /// 由于base_link 到 utm或者ENU的旋转部分这里没法获得，还是直接返回GPS的 UTM或者ENU坐标好了
//        //        tf2::Transform robot_orientation;
//        //        can_transform = lookupTransformSafe(tf_buffer_,
//        //                                            map_frame_id_,
//        //                                            base_link_frame_id_,
//        //                                            transform_time,
//        //                                            transform_timeout_,
//        //                                            robot_orientation,
//        //                                            false);

//        //        if (can_transform)
//        //        {
//        //            // Zero out rotation because we don't care about the orientation of the
//        //            // GPS receiver relative to base_link
//        //            // robot_orientation.getRotation() ： base_link 到 utm或者ENU坐标系的旋转
//        //            // gps_offset_rotated : 在这里变成了 gps传感器坐标系 到 utm 或者 ENU坐标系的变换
//        //            gps_offset_rotated.setOrigin(tf2::quatRotate(robot_orientation.getRotation(), gps_offset_rotated.getOrigin()));
//        //            gps_offset_rotated.setRotation(tf2::Quaternion::getIdentity());
//        //            // gps_output_transform_： gps传感器坐标系 到 utm 或者 ENU坐标系的变换
//        //            // gps_offset_rotated ： gps传感器坐标系 到 utm 或者 ENU坐标系的变换
//        //            // gps_offset_rotated.inverse(): utm 或者 ENU坐标系  到 gps传感器坐标系的变换
//        //            ///base_link_output_transform:
//        //            base_link_output_transform = gps_offset_rotated.inverse() * gps_output_transform_;
//        //        }
//        //        else
//        //        {
//        //            ROS_WARN_STREAM_THROTTLE(5.0, "Could not obtain " << map_frame_id_ << "->" << base_link_frame_id_ <<
//        //                                     " transform. Will not remove offset of navsat device from robot's origin.");
//        //        }
//    }
//    else
//    {
//        ROS_WARN_STREAM_THROTTLE(5.0, "Could not obtain " << base_link_frame_id_ << "->" << gps_frame_id_ <<
//                                 " transform. Will not remove offset of navsat device from robot's origin.");
//    }
//}

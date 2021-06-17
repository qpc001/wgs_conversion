
#include <Eigen/Geometry>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include "tf_listener.hpp"

TFListener::TFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id) 
    :nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id) {
}

bool TFListener::LookupData(Eigen::Matrix4f& transform_matrix) {
    try {
        tf::StampedTransform transform;
        listener_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0), transform);
        TransformToMatrix(transform, transform_matrix);
        return true;
    } catch (tf::TransformException &ex) {
        return false;
    }
}

bool TFListener::TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix) {
    Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
    
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());

    //tf2::fromMsg(transform,transform_matrix);

    Eigen::Quaternionf qf;
    qf.w()=transform.getRotation().w();
    qf.x()=transform.getRotation().x();
    qf.y()=transform.getRotation().y();
    qf.z()=transform.getRotation().z();
    Eigen::Vector3f t_;
    t_.x()=transform.getOrigin().x();
    t_.y()=transform.getOrigin().y();
    t_.z()=transform.getOrigin().z();

    // TEST
    transform_matrix.block(0,0,3,3)=qf.matrix();
    transform_matrix.block(0,3,3,1)=t_;

    // ROS_INFO("qw: %f , qx: %f, qy: %f, qz: %f",qf.w(),qf.x(),qf.y(),qf.z());
    // std::cout<<qf.matrix()<<std::endl;
    // 此矩阵为 child_frame_id 到 base_frame_id 的转换矩阵
    // transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

    return true;
}


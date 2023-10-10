// Copyright (c) 2023 Leo Drive Teknoloji A.Ş.

#include <memory>
#include <residual_analiz/residual_analiz.hpp>
#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

ResidualAnaliz::ResidualAnaliz()
    : Node("ResidualAnaliz") {

    pub_absolute_error = this->create_publisher<std_msgs::msg::Float32>("/analyzer/position_difference/absolute_error_xy", 10);
    pub_difference_xyz_yaw = this->create_publisher<std_msgs::msg::Float64MultiArray>("/analyzer/difference/xyz_yaw",10);
    pub_pose_with_only_twist = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pub_pose_with_only_twist",10);

    sub_twist_sync.subscribe(this,"/localization/pose_twist_fusion_filter/biased_pose_only_velocity",rclcpp::QoS{1}.get_rmw_qos_profile());
    sub_aw_ekf_sync.subscribe(this, "/localization/pose_twist_fusion_filter/biased_pose", rclcpp::QoS{1}.get_rmw_qos_profile());

    sync_ptr_.reset(new ExactTimeSynchronizer(ExactTimeSyncPolicy(10), sub_aw_ekf_sync, sub_twist_sync));

    sync_ptr_->registerCallback(
            std::bind(&ResidualAnaliz::testCallback, this, std::placeholders::_1, std::placeholders::_2));
    std::cout<<"sadkjsjdhfsa"<<std::endl;
}

void ResidualAnaliz::testCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &in_aw_ekf_msg,
                                  const geometry_msgs::msg::PoseStamped::ConstSharedPtr &in_twist_msg) {


    tf2::Quaternion aw_ekf_quaternion;
    tf2::fromMsg(in_aw_ekf_msg->pose.orientation, aw_ekf_quaternion);

    double roll_aw_, pitch_aw_, yaw_aw_;
    tf2::Matrix3x3(aw_ekf_quaternion).getRPY(roll_aw_, pitch_aw_, yaw_aw_);

    //create Twist only posewithcovstampedmsg
    geometry_msgs::msg::PoseWithCovarianceStamped ekf_with_only_twist;
    ekf_with_only_twist.header = in_twist_msg->header;
    ekf_with_only_twist.pose.pose = in_twist_msg->pose;

    double position_difference_x = abs(in_aw_ekf_msg->pose.position.x - in_twist_msg->pose.position.x);
    double position_difference_y = abs(in_aw_ekf_msg->pose.position.y - in_twist_msg->pose.position.y);
    double position_difference_z = abs(in_aw_ekf_msg->pose.position.z - in_twist_msg->pose.position.z);

    ekf_with_only_twist.pose.covariance[0]  = std::pow(position_difference_x,2);
    ekf_with_only_twist.pose.covariance[7]  = std::pow(position_difference_y,2);
    ekf_with_only_twist.pose.covariance[14] = std::pow(position_difference_z,2);


    tf2::Quaternion twist_quaternion;
    tf2::fromMsg(in_twist_msg->pose.orientation, twist_quaternion);

    double roll_twist_only_, pitch_twist_only_, yaw_twist_only_;
    tf2::Matrix3x3(twist_quaternion).getRPY(roll_twist_only_, pitch_twist_only_, yaw_twist_only_);

    double roll_difference_in_radians  = abs(roll_twist_only_ - roll_aw_);
    double pitch_difference_in_radians = abs(pitch_twist_only_ - pitch_aw_);
    double yaw_difference_in_radians   = abs(yaw_twist_only_ - yaw_aw_);

    ekf_with_only_twist.pose.covariance[21] = std::pow(roll_difference_in_radians, 2);
    ekf_with_only_twist.pose.covariance[28] = std::pow(pitch_difference_in_radians, 2);
    ekf_with_only_twist.pose.covariance[35] = std::pow(yaw_difference_in_radians, 2);

    pub_pose_with_only_twist->publish(ekf_with_only_twist);

    auto difference_ = std::make_unique<std_msgs::msg::Float64MultiArray>();

    difference_->data = {position_difference_x,position_difference_y,position_difference_z,(yaw_difference_in_radians * 2 * M_PI)};
//    std_msgs::msg::Float64MultiArray difference_;
//    difference_.data[0] = position_difference_x;
//    difference_.data[1] = position_difference_y;
//    difference_.data[2] = position_difference_z;
//    difference_.data[3] = yaw_difference_in_radians;

    pub_difference_xyz_yaw->publish(std::move(difference_));

    std_msgs::msg::Float32 absolute_error_msg;
    absolute_error_msg.data = sqrt(std::pow((position_difference_x), 2) +
                                   std::pow((position_difference_y), 2));

    pub_absolute_error->publish(absolute_error_msg);

}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ResidualAnaliz>());
  rclcpp::shutdown();
  return 0;
}

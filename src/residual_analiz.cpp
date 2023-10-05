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

    publisher_x_diff_ = this->create_publisher<std_msgs::msg::Float32>("residual_x", 10);
    publisher_y_diff_ = this->create_publisher<std_msgs::msg::Float32>("residual_y", 10);
    pub_pose_with_only_ndt = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pub_pose_with_only_ndt",10);
    pub_pose_with_only_twist = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pub_pose_with_only_twist",10);

    sub_ndt_sync.subscribe(this,"/localization/pose_twist_fusion_filter/biased_pose_only_ndt",rclcpp::QoS{1}.get_rmw_qos_profile());
    sub_twist_sync.subscribe(this,"/localization/pose_twist_fusion_filter/biased_pose_only_velocity",rclcpp::QoS{1}.get_rmw_qos_profile());
    sub_aw_ekf_sync.subscribe(this, "/localization/pose_twist_fusion_filter/biased_pose", rclcpp::QoS{1}.get_rmw_qos_profile());

    sync_ptr_.reset(new ExactTimeSynchronizer(ExactTimeSyncPolicy(10), sub_ndt_sync, sub_aw_ekf_sync, sub_twist_sync));

    sync_ptr_->registerCallback(
            std::bind(&ResidualAnaliz::testCallback, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3));
    std::cout<<"sadkjsjdhfsa"<<std::endl;
}

void ResidualAnaliz::testCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &in_ndt_msg,
                                  const geometry_msgs::msg::PoseStamped::ConstSharedPtr &in_aw_ekf_msg,
                                  const geometry_msgs::msg::PoseStamped::ConstSharedPtr &in_twist_msg) {

    std_msgs::msg::Float32 diff_x, diff_y;
    diff_x.data = abs(in_aw_ekf_msg->pose.position.x - in_ndt_msg->pose.position.x);
    diff_y.data = abs(in_aw_ekf_msg->pose.position.y - in_ndt_msg->pose.position.y);
    publisher_x_diff_->publish(diff_x);
    publisher_y_diff_->publish(diff_y);


    tf2::Quaternion aw_ekf_quaternion;
    tf2::fromMsg(in_aw_ekf_msg->pose.orientation, aw_ekf_quaternion);

    double roll_aw_, pitch_aw_, yaw_aw_;
    tf2::Matrix3x3(aw_ekf_quaternion).getRPY(roll_aw_, pitch_aw_, yaw_aw_);

    geometry_msgs::msg::PoseWithCovarianceStamped ekf_with_only_ndt;

    //create NDT only posewithcovstampedmsg
    ekf_with_only_ndt.header = in_ndt_msg->header;
    ekf_with_only_ndt.pose.pose = in_ndt_msg->pose;
    ekf_with_only_ndt.pose.covariance[0]  = abs(in_aw_ekf_msg->pose.position.x - in_ndt_msg->pose.position.x);
    ekf_with_only_ndt.pose.covariance[7]  = abs(in_aw_ekf_msg->pose.position.y - in_ndt_msg->pose.position.y);
    ekf_with_only_ndt.pose.covariance[14] = abs(in_aw_ekf_msg->pose.position.z - in_ndt_msg->pose.position.z);


    tf2::Quaternion ndt_quaternion;
    tf2::fromMsg(in_ndt_msg->pose.orientation, ndt_quaternion);

    double roll_ndt_only_, pitch_ndt_only_, yaw_ndt_only_;
    tf2::Matrix3x3(ndt_quaternion).getRPY(roll_ndt_only_, pitch_ndt_only_, yaw_ndt_only_);

    ekf_with_only_ndt.pose.covariance[21] = abs(roll_ndt_only_  - roll_aw_);
    ekf_with_only_ndt.pose.covariance[28] = abs(pitch_ndt_only_ - pitch_aw_);
    ekf_with_only_ndt.pose.covariance[35] = abs(yaw_ndt_only_   - yaw_aw_);



    //create Twist only posewithcovstampedmsg

    geometry_msgs::msg::PoseWithCovarianceStamped ekf_with_only_twist;
    ekf_with_only_twist.header = in_twist_msg->header;
    ekf_with_only_twist.pose.pose = in_twist_msg->pose;
    ekf_with_only_twist.pose.covariance[0]  = abs(in_aw_ekf_msg->pose.position.x - in_twist_msg->pose.position.x);
    ekf_with_only_twist.pose.covariance[7]  = abs(in_aw_ekf_msg->pose.position.y - in_twist_msg->pose.position.y);
    ekf_with_only_twist.pose.covariance[14] = abs(in_aw_ekf_msg->pose.position.z - in_twist_msg->pose.position.z);


    tf2::Quaternion twist_quaternion;
    tf2::fromMsg(in_twist_msg->pose.orientation, twist_quaternion);

    double roll_twist_only_, pitch_twist_only_, yaw_twist_only_;
    tf2::Matrix3x3(twist_quaternion).getRPY(roll_twist_only_, pitch_twist_only_, yaw_twist_only_);

    ekf_with_only_twist.pose.covariance[21] = abs(roll_twist_only_  - roll_aw_);
    ekf_with_only_twist.pose.covariance[28] = abs(pitch_twist_only_ - pitch_aw_);
    ekf_with_only_twist.pose.covariance[35] = abs(yaw_twist_only_   - yaw_aw_);

    std::cout<<"YAW ANGLE ONLY TWIST : "<< yaw_twist_only_  <<std::endl;
    std::cout<<"YAW ANGLE AW ekf     : "<< yaw_aw_<<std::endl;

    pub_pose_with_only_ndt->publish(ekf_with_only_ndt);
    pub_pose_with_only_twist->publish(ekf_with_only_twist);


}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ResidualAnaliz>());
  rclcpp::shutdown();
  return 0;
}

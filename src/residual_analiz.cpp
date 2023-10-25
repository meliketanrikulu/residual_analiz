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

    pub_absolute_error_dr2 = this->create_publisher<std_msgs::msg::Float32>("/analyzer/position_difference/dr2/EuclideanDistance_xy", 10);
    pub_absolute_error_dr1 = this->create_publisher<std_msgs::msg::Float32>("/analyzer/position_difference/dr1/EuclideanDistance_xy", 10);

    pub_difference_dr1_xyz_yaw = this->create_publisher<std_msgs::msg::Float64MultiArray>("/analyzer/difference/dr1/xyz_yaw", 10);
    pub_difference_dr2_xyz_yaw = this->create_publisher<std_msgs::msg::Float64MultiArray>("/analyzer/difference/dr2/xyz_yaw", 10);

    pub_pose_with_only_dr1 = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pub_pose_dr1", 10);
    pub_pose_with_with_dr2 = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pub_pose_dr2", 10);

    pub_ndt_status =this->create_publisher<std_msgs::msg::String>("/ndt_status",10);

    sub_aw_ekf_sync.subscribe(this, "/localization/pose_twist_fusion_filter/pose", rclcpp::QoS{1}.get_rmw_qos_profile());
    sub_ekf_dr_1_.subscribe(this, "/localization/pose_twist_fusion_filter/ekf_pose_dr", rclcpp::QoS{1}.get_rmw_qos_profile());
    sub_ekf_dr_2_.subscribe(this, "/localization/pose_twist_fusion_filter/ekf_pose_dr_2_", rclcpp::QoS{1}.get_rmw_qos_profile());

    sync_ptr_.reset(new ExactTimeSynchronizer(ExactTimeSyncPolicy(10), sub_aw_ekf_sync, sub_ekf_dr_1_, sub_ekf_dr_2_));

    sync_ptr_->registerCallback(
            std::bind(&ResidualAnaliz::testCallback, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3));
    std::cout<<"sadkjsjdhfsa"<<std::endl;
}

void ResidualAnaliz::testCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &in_aw_ekf_msg,
                                  const geometry_msgs::msg::PoseStamped::ConstSharedPtr &in_ekf_dr_1_,
                                  const geometry_msgs::msg::PoseStamped::ConstSharedPtr &in_ekf_dr_2_) {


    tf2::Quaternion aw_ekf_quaternion;
    tf2::fromMsg(in_aw_ekf_msg->pose.orientation, aw_ekf_quaternion);

    double roll_aw_, pitch_aw_, yaw_aw_;
    tf2::Matrix3x3(aw_ekf_quaternion).getRPY(roll_aw_, pitch_aw_, yaw_aw_);

    //create dr1 posewithcovstampedmsg
    geometry_msgs::msg::PoseWithCovarianceStamped ekf_dr1_;
    ekf_dr1_.header = in_ekf_dr_1_->header;
    ekf_dr1_.pose.pose = in_ekf_dr_1_->pose;

    double position_difference_x = abs(in_aw_ekf_msg->pose.position.x - in_ekf_dr_1_->pose.position.x);
    double position_difference_y = abs(in_aw_ekf_msg->pose.position.y - in_ekf_dr_1_->pose.position.y);
    double position_difference_z = abs(in_aw_ekf_msg->pose.position.z - in_ekf_dr_1_->pose.position.z);

    ekf_dr1_.pose.covariance[0]  = std::pow(position_difference_x, 2);
    ekf_dr1_.pose.covariance[7]  = std::pow(position_difference_y, 2);
    ekf_dr1_.pose.covariance[14] = std::pow(position_difference_z, 2);


    tf2::Quaternion dr1_quaternion;
    tf2::fromMsg(in_ekf_dr_1_->pose.orientation, dr1_quaternion);

    double roll_dr1_only_, pitch_dr1_only_, yaw_dr1_only_;
    tf2::Matrix3x3(dr1_quaternion).getRPY(roll_dr1_only_, pitch_dr1_only_, yaw_dr1_only_);

    double roll_difference_in_radians  = abs(roll_dr1_only_ - roll_aw_);
    double pitch_difference_in_radians = abs(pitch_dr1_only_ - pitch_aw_);
    double yaw_difference_in_radians;
    if ((yaw_dr1_only_ > 1.5708 && yaw_aw_ < -1.5708) || (yaw_dr1_only_ < -1.5708 && yaw_aw_ > 1.5708)){
        yaw_difference_in_radians = 6.28319 - abs(yaw_dr1_only_ - yaw_aw_);
    }
    else {
        yaw_difference_in_radians = abs(yaw_dr1_only_ - yaw_aw_);
    }
    ekf_dr1_.pose.covariance[21] = std::pow(roll_difference_in_radians, 2);
    ekf_dr1_.pose.covariance[28] = std::pow(pitch_difference_in_radians, 2);
    ekf_dr1_.pose.covariance[35] = std::pow(yaw_difference_in_radians, 2);

    pub_pose_with_only_dr1->publish(ekf_dr1_);

    auto difference_ = std::make_unique<std_msgs::msg::Float64MultiArray>();

    difference_->data = {position_difference_x,position_difference_y,position_difference_z,(yaw_difference_in_radians * 180 / M_PI)};
    if((yaw_difference_in_radians * 180 / M_PI) >= 300.0){
        std::cout << "yaw_dr1_only_ : " << yaw_dr1_only_ * 180 / M_PI << std::endl;
        std::cout<<"yaw_aw_ : "<<yaw_aw_ * 180 / M_PI <<std::endl;
        std::cout<<"yaw_difference_in_radians : : : "<<yaw_difference_in_radians<<std::endl;
    }


    pub_difference_dr1_xyz_yaw->publish(std::move(difference_));

    std_msgs::msg::Float32 absolute_error_dr1_msg;
    absolute_error_dr1_msg.data = sqrt(std::pow((position_difference_x), 2) +
                                       std::pow((position_difference_y), 2));

    pub_absolute_error_dr1->publish(absolute_error_dr1_msg);


    //2. compare pose

//    tf2::Quaternion aw_ekf_quaternion;
//    tf2::fromMsg(in_aw_ekf_msg->pose.orientation, aw_ekf_quaternion);
//
//    double roll_aw_, pitch_aw_, yaw_aw_;
//    tf2::Matrix3x3(aw_ekf_quaternion).getRPY(roll_aw_, pitch_aw_, yaw_aw_);

    //create dr1 only posewithcovstampedmsg
    geometry_msgs::msg::PoseWithCovarianceStamped ekf_dr2_;
    ekf_dr2_.header = in_ekf_dr_2_->header;
    ekf_dr2_.pose.pose = in_ekf_dr_2_->pose;

    double position_difference_dr2_x = abs(in_aw_ekf_msg->pose.position.x - in_ekf_dr_2_->pose.position.x);
    double position_difference_dr2_y = abs(in_aw_ekf_msg->pose.position.y - in_ekf_dr_2_->pose.position.y);
    double position_difference_dr2_z = abs(in_aw_ekf_msg->pose.position.z - in_ekf_dr_2_->pose.position.z);

    ekf_dr2_.pose.covariance[0]  = std::pow(position_difference_dr2_x, 2);
    ekf_dr2_.pose.covariance[7]  = std::pow(position_difference_dr2_y, 2);
    ekf_dr2_.pose.covariance[14] = std::pow(position_difference_dr2_z, 2);

    tf2::Quaternion dr2_quaternion;
    tf2::fromMsg(in_ekf_dr_2_->pose.orientation, dr2_quaternion);

    double roll_with_dr2_, pitch_with_dr2_, yaw_with_dr2_;
    tf2::Matrix3x3(dr2_quaternion).getRPY(roll_with_dr2_, pitch_with_dr2_, yaw_with_dr2_);

    double roll_difference_in_radians_dr2_  = abs(roll_with_dr2_ - roll_aw_);
    double pitch_difference_in_radians_dr2_ = abs(pitch_with_dr2_ - pitch_aw_);
    double yaw_difference_in_radians_dr2_;
    if ((yaw_with_dr2_ > 1.5708 && yaw_aw_ < -1.5708) || (yaw_with_dr2_ < -1.5708 && yaw_aw_ > 1.5708)){
        yaw_difference_in_radians_dr2_ = 6.28319 - abs(yaw_with_dr2_ - yaw_aw_);
    }
    else {
        yaw_difference_in_radians_dr2_ = abs(yaw_with_dr2_ - yaw_aw_);
    }
    ekf_dr2_.pose.covariance[21] = std::pow(roll_difference_in_radians_dr2_, 2);
    ekf_dr2_.pose.covariance[28] = std::pow(pitch_difference_in_radians_dr2_, 2);
    ekf_dr2_.pose.covariance[35] = std::pow(yaw_difference_in_radians_dr2_, 2);

    pub_pose_with_with_dr2->publish(ekf_dr2_);

    auto difference_dr2_ = std::make_unique<std_msgs::msg::Float64MultiArray>();

    difference_dr2_->data = {position_difference_dr2_x, position_difference_dr2_y, position_difference_dr2_z, (yaw_difference_in_radians_dr2_ * 180 / M_PI)};
    if((yaw_difference_in_radians_dr2_ * 180 / M_PI) >= 300.0){
        std::cout << "yaw_dr1_only_ : " << yaw_with_dr2_ * 180 / M_PI << std::endl;
        std::cout<<"yaw_aw_ : "<<yaw_aw_ * 180 / M_PI <<std::endl;
        std::cout << "yaw_difference_in_radians_dr2_ : : : " << yaw_difference_in_radians_dr2_ << std::endl;
    }


    pub_difference_dr2_xyz_yaw->publish(std::move(difference_dr2_));

    std_msgs::msg::Float32 absolute_error_dr2_msg;
    absolute_error_dr2_msg.data = sqrt(std::pow((position_difference_dr2_x), 2) +
                                       std::pow((position_difference_dr2_y), 2));

    pub_absolute_error_dr2->publish(absolute_error_dr2_msg);


    if (absolute_error_dr1_msg.data >= 1.0 || absolute_error_dr2_msg.data >= 1.0){
        std_msgs::msg::String ndt_status_;
        ndt_status_.data = "NDT FAILED";
        pub_ndt_status->publish(ndt_status_);
    }

}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ResidualAnaliz>());
  rclcpp::shutdown();
  return 0;
}

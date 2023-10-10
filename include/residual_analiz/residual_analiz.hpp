// Copyright (c) 2022 Leo Drive Teknoloji A.Ş.

#ifndef RESIDUAL_ANALIZ__RESIDUAL_ANALIZ_H
#define RESIDUAL_ANALIZ__RESIDUAL_ANALIZ_H


#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <std_msgs/msg/float32.hpp>
#include <memory>

class ResidualAnaliz : public rclcpp::Node {
public:
    ResidualAnaliz();
    using pose_msg = geometry_msgs::msg::PoseStamped;
    using ExactTimeSyncPolicy = message_filters::sync_policies::ApproximateTime<pose_msg, pose_msg>;
    using ExactTimeSynchronizer = message_filters::Synchronizer<ExactTimeSyncPolicy>;
    std::shared_ptr<ExactTimeSynchronizer> sync_ptr_;

    message_filters::Subscriber<pose_msg> sub_twist_sync;
    message_filters::Subscriber<pose_msg> sub_aw_ekf_sync;
private:
    void testCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & in_aw_ekf_msg,
                      const geometry_msgs::msg::PoseStamped::ConstSharedPtr & sub_twist_sync) ;


    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_with_only_twist;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_absolute_error;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_difference_xyz_yaw;
};

#endif  // RESIDUAL_ANALIZ__RESIDUAL_ANALIZ_H

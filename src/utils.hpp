#ifndef UTILS_HPP
#define UTILS_HPP

#include <rclcpp/qos.hpp>
#include <rmw/types.h>

rclcpp::QoS getQOS(rmw_qos_reliability_policy_t reliability, rmw_qos_history_policy_t history, int depth, rclcpp::Duration deadline) {
    auto qos = rclcpp::QoS(depth);
    qos.reliability(reliability);
    qos.history(history);
    qos.deadline(deadline);  // RT deadline
    qos.lifespan(rclcpp::Duration::from_seconds(1.0));  // 寿命调优
    return qos;
}

#endif
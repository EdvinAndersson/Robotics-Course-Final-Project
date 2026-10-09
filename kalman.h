#pragma once

#include <cmath>
#include "kalman.h"

inline Pose2DWithCov kalman_predict(const Pose2DWithCov& odom) {
    // In this simplified model, prediction just forwards odometry
    kalman_pose = odom;
    return kalman_pose;
}

inline Pose2DWithCov kalman_update(const Pose2DWithCov& odom, const Pose2DWithCov& scan) {
    // Custom weights
    const float w_odom_x   = 0.8f;
    const float w_scan_x   = 1.0f - w_odom_x;

    const float w_scan_y   = 0.2f;
    const float w_odom_y   = 1.0f - w_scan_y;

    const float w_theta    = 0.5f;  // equal trust
    const float w_scan_th  = w_theta;
    const float w_odom_th  = 1.0f - w_theta;

    kalman_pose.pose.x     = w_odom_x  * odom.pose.x   + w_scan_x  * scan.pose.x;
    kalman_pose.pose.y     = w_odom_y  * odom.pose.y   + w_scan_y  * scan.pose.y;
    kalman_pose.pose.theta = w_odom_th * -odom.pose.theta + w_scan_th * scan.pose.theta;

    kalman_pose.sigma_x = (odom.sigma_x + scan.sigma_x) / 2.0f;
    kalman_pose.sigma_y = (odom.sigma_y + scan.sigma_y) / 2.0f;
    kalman_pose.sigma_theta = (odom.sigma_theta + scan.sigma_theta) / 2.0f;

    return kalman_pose;
}

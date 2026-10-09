#include "odometry.h"
#include <cmath>

// === Robot-specific constants ===
const float WHEEL_RADIUS = 0.015f;       // 15 mm radius
const float WHEEL_BASE = 0.14f;        // 140 mm between wheels
const int TICKS_PER_REV = 1024 * 13 * 4;  // ticks per full wheel revolution
const float ticks_to_meters = 0.255 * M_PI * WHEEL_RADIUS / TICKS_PER_REV;

Pose2DWithCov computeOdometryDelta(
    int left_ticks, int right_ticks,
    int last_left_ticks, int last_right_ticks,
    double encoder_noise_std) // in meters
{
    // Tick differences
    int delta_left_ticks  = -(left_ticks - last_left_ticks);
    int delta_right_ticks = (right_ticks - last_right_ticks);

    // Convert to distance
    double d_left  = delta_left_ticks  * ticks_to_meters;
    double d_right = delta_right_ticks * ticks_to_meters;

    // Compute delta motion in robot frame
    double d_center = (d_right + d_left) / 2.0;
    double d_theta  = (d_right - d_left) / WHEEL_BASE;

    Pose2DWithCov odom;

    // Pose in robot frame: x forward, y sideways (usually 0), theta rotation
    odom.pose.x = d_center;
    odom.pose.y = 0.0;
    odom.pose.theta = -d_theta;

    // Estimated uncertainty (tune or model based on system)
    odom.sigma_x = std::abs(d_center * 0.02) + encoder_noise_std;  // 2% of distance
    odom.sigma_y = 0.01;  // Small lateral uncertainty
    odom.sigma_theta = std::abs(d_theta * 0.01) + 0.01; // 10% of angle, min 0.01 rad

    return odom;
}
#if 0
Pose2DWithCov compute_odometry(
    int enc_right_now, int enc_left_now,
    int enc_right_prev, int enc_left_prev,
    float x_prev, float y_prev, float theta_prev
) {
    Pose2DWithCov pose_cov;

    // Calculate encoder tick deltas
    int delta_R_ticks = enc_right_now - enc_right_prev;
    int delta_L_ticks = -(enc_left_now - enc_left_prev);  // left is reversed

    float dR = delta_R_ticks * ticks_to_meters;
    float dL = delta_L_ticks * ticks_to_meters;

    // Odometry equations
    float dS = (dR + dL) / 2.0f;
    float dTheta = (dR - dL) / WHEEL_BASE;

    // Update pose
    pose_cov.pose.x = x_prev + dS * cos(theta_prev + dTheta / 2.0f);
    pose_cov.pose.y = y_prev - dS * sin(theta_prev + dTheta / 2.0f);
    pose_cov.pose.theta = theta_prev + dTheta;
    pose_cov.pose.theta = atan2f(sinf(pose_cov.pose.theta), cosf(pose_cov.pose.theta));

    // Simple uncertainty model
    pose_cov.sigma_x = fabs(dS) * 0.01f;
    pose_cov.sigma_y = fabs(dS) * 0.01f;
    pose_cov.sigma_theta = fabs(dTheta) * 0.05f;

    return pose_cov;
}
#endif
#pragma once

#include "odometry.h"
#include <cmath>
#include "scan_match.h"

Pose2DWithCov computeOdometryDelta(
    int left_ticks, int right_ticks,
    int last_left_ticks, int last_right_ticks,
    double encoder_noise_std
);
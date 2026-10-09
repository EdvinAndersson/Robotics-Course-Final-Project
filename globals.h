#pragma once

#include "lidar.h"
#include "camera.h"
#include <mutex>

extern int running;

extern int frame_width;
extern int frame_height;
extern Box target_box;

extern LidarData lidar_data;
extern std::mutex mutex;

extern Pose2DWithCov kalman_pose;
extern Pose2DWithCov initial_pose;
#pragma once

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "scan_match.h"

#define PORT 9888
#define PORT_START 9887
#define HEADER_SIZE 5
#define DATA_BUFFER_SIZE 4096

struct LidarScan {
    Pose2D pose;
    Pose2DWithCov result;
};

struct LidarData {
    int lidar_connected;
    int has_scanned;
    int is_scanning;
    int has_scanned_once;
    LidarScan current_lidar_scan;
};

void* lidar_start_receiving(void *arg);
void* lidar_listen(void *arg);
int lidar_running(int running);
void lidar_shutdown();
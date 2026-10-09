#pragma once

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

struct Box {
    cv::Rect rect;
    double pixel_area;
    int dst_from_center;
    int y;
};

void* camera_thread(void *arg);
void camera_release();
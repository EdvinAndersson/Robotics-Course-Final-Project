#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Dense>

struct Pose2D {
    double x, y, theta;
};
struct Pose2DWithCov {
    Pose2D pose;
    double sigma_x;
    double sigma_y;
    double sigma_theta; // in radians
};
struct Line {
    Eigen::Vector2d p1;
    Eigen::Vector2d p2;
};

struct ScanPoint {
    double angle_rad;
    double distance;
    double quality;
};

struct LineData {
    Eigen::Vector2d u;
    double r;
};

inline Line make_line(int x1, int y1, int x2, int y2) {
    Line l;
    l.p1[0] = x1;
    l.p1[1] = y1;
    l.p2[0] = x2;
    l.p2[1] = y2;
    return l;
}

Pose2DWithCov scan_match(const std::vector<ScanPoint>& scan, const std::vector<Line>& map_lines, const Pose2D& initial_pose);
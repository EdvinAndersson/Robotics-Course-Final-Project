#include "scan_match.h"

Pose2DWithCov scan_match(const std::vector<ScanPoint>& scan, const std::vector<Line>& map_lines, const Pose2D& initial_pose) {
    Pose2D pose = initial_pose;
    Eigen::Matrix3d covariance;

    // Step 0 – Precompute ui and ri for all lines
    std::vector<LineData> line_data;
    for (const auto& l : map_lines) {
        Eigen::Vector2d dir = l.p2 - l.p1;
        Eigen::Vector2d u = dir.normalized();
        double r = u.dot(l.p1);  // Distance from origin to line
        line_data.push_back({u, r});
    }

    const int max_iterations = 20;
    double dx_total = 0, dy_total = 0, dtheta_total = 0;

    for (int iter = 0; iter < max_iterations; ++iter) {
        std::vector<Eigen::Vector3d> A_rows;
        std::vector<double> y_vec;

        Eigen::Vector2d vm_sum(0, 0);
        std::vector<Eigen::Vector2d> v_world;

        // Step 1 – Transform scan points into world coordinates
        for (const auto& p : scan) {
            double x_sensor = p.distance * cos(p.angle_rad);
            double y_sensor = p.distance * sin(p.angle_rad);

            // Sensor -> robot -> world
            double cs = cos(pose.theta);
            double sn = sin(pose.theta);
            Eigen::Vector2d world_point;
            world_point.x() = cs * x_sensor - sn * y_sensor + pose.x;
            world_point.y() = sn * x_sensor + cs * y_sensor + pose.y;

            vm_sum += world_point;
            v_world.push_back(world_point);
        }

        if (v_world.empty()) return Pose2DWithCov{{-1,-1, -1}, -1,-1,-1};
        Eigen::Vector2d vm = vm_sum / (double) v_world.size();

        // Step 2 – Assign each point to closest line
        for (size_t i = 0; i < v_world.size(); ++i) {
            const Eigen::Vector2d& vi = v_world[i];

            double min_yi = std::numeric_limits<double>::max();
            int best_j = -1;

            for (size_t j = 0; j < line_data.size(); ++j) {
                const auto& line = line_data[j];
                double yi = line.r - line.u.dot(vi);
                if (std::abs(yi) < std::abs(min_yi)) {
                    min_yi = yi;
                    best_j = j;
                }
            }

            if (best_j == -1 || std::abs(min_yi) > 0.05) continue;  // Outlier rejection

            const auto& line = line_data[best_j];

            double xi1 = line.u.x();
            double xi2 = line.u.y();

            // Rotate (vi - vm) by 90 deg CCW
            Eigen::Vector2d rot = Eigen::Vector2d(-(vi - vm).y(), (vi - vm).x());
            double xi3 = line.u.dot(rot);

            A_rows.emplace_back(xi1, xi2, xi3);
            y_vec.push_back(min_yi);
        }

        if (A_rows.size() < 4){
	     return Pose2DWithCov{{-1,-1, -1}, -1,-1,-1};// Not enough points
	}

        // Step 3 – Least-squares solution
        Eigen::MatrixXd A(A_rows.size(), 3);
        Eigen::VectorXd Y(y_vec.size());
        for (size_t i = 0; i < A_rows.size(); ++i) {
            A.row(i) = A_rows[i];
            Y(i) = y_vec[i];
        }

        Eigen::Vector3d B = (A.transpose() * A).ldlt().solve(A.transpose() * Y);

        double dx = B(0);
        double dy = B(1);
        double dtheta = B(2);

        dx_total += dx;
        dy_total += dy;
        dtheta_total += dtheta;

        pose.x += dx;
        pose.y += dy;
        pose.theta += dtheta;

        // Step 5 – Convergence check
        if (iter == max_iterations - 1 || std::sqrt(dx*dx + dy*dy) < 0.05 && std::abs(dtheta) < 1.5 * M_PI / 180.0) {
            // Estimate residual variance (mean squared error)
            double residual_sum_sq = (A * B - Y).squaredNorm();
            double sigma2 = residual_sum_sq / static_cast<double>(A_rows.size() - 3); // DoF = N - 3
		Eigen::Matrix3d AtA = A.transpose() * A;
		if (AtA.determinant() < 1e-10) {
		    // Ill-conditioned, return high uncertainty
		    return Pose2DWithCov{pose, 9999, 9999, 9999};
		}	    
		covariance = sigma2 * (A.transpose() * A).inverse();
	    break;
        }
    }
    double std_x = std::sqrt(covariance(0, 0));
    double std_y = std::sqrt(covariance(1, 1));
    double std_theta = std::sqrt(covariance(2, 2));
    
    Pose2DWithCov result;
    result.pose = pose;
    result.sigma_x = std_x;
    result.sigma_y = std_y;
    result.sigma_theta = std_theta;
    return result;
}
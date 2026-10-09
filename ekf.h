#include <iostream>
#include <Eigen/Dense>
#include <cmath>

class EKF {
    public:
        EKF() {}
    inline void init(const Pose2DWithCov& initial_state) {
        // Set state vector [x, y, theta]
        x_(0) = initial_state.pose.x,
        x_(1) = initial_state.pose.y,
        x_(2) = initial_state.pose.theta;

        // Set covariance matrix (assumes diagonal only)
        P_ = Eigen::Matrix3d::Zero();
        P_(0, 0) = initial_state.sigma_x * initial_state.sigma_x;
        P_(1, 1) = initial_state.sigma_y * initial_state.sigma_y;
        P_(2, 2) = initial_state.sigma_theta * initial_state.sigma_theta;

    }
    inline void predict(const Pose2DWithCov& odom) {
        // Extract motion delta
        double dx = odom.pose.x;
        double dy = odom.pose.y;
        double dtheta = odom.pose.theta;
    
        // Convert sigma values to covariance matrix Q
        //Eigen::Matrix3d Q = toCovarianceMatrix(odom);
      
        Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
        Q(0, 0) = 0.001;   // small position noise
        Q(1, 1) = 0.001;
        Q(2, 2) = 0.0001;  // small angle noise
        // Current orientation
        double theta = x_(2);
        double cos_theta = std::cos(theta);
        double sin_theta = std::sin(theta);
    
        // Predict new state (apply motion in robot frame)
        x_(0) += cos_theta * dx - sin_theta * dy;
        x_(1) += sin_theta * dx + cos_theta * dy;
        x_(2) += dtheta;
    
        // Normalize theta
        x_(2) = std::atan2(std::sin(x_(2)), std::cos(x_(2)));
    
        // Jacobian of motion model w.r.t state
        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        F(0, 2) = -sin_theta * dx - cos_theta * dy;
        F(1, 2) =  cos_theta * dx - sin_theta * dy;
    
        // Predict new covariance
        P_ = F * P_ * F.transpose() + Q;
    }

    inline void update(const Pose2DWithCov& z) {
        Eigen::Vector3d z_vec = toVector3d(z);
        Eigen::Matrix3d R = toCovarianceMatrix(z);
    
        Eigen::Vector3d y = z_vec - x_;
        y(2) = std::atan2(std::sin(y(2)), std::cos(y(2)));
    
        Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d S = H * P_ * H.transpose() + R;
    
        Eigen::Matrix3d K = P_ * H.transpose() * S.inverse();
        x_ = x_ + K * y;
        x_(2) = std::atan2(std::sin(x_(2)), std::cos(x_(2)));
    
        P_ = (Eigen::Matrix3d::Identity() - K * H) * P_;
    }


    inline Eigen::Vector3d toVector3d(const Pose2DWithCov& p) {
        return Eigen::Vector3d(p.pose.x, p.pose.y, p.pose.theta);
    }
    
    inline Eigen::Matrix3d toCovarianceMatrix(const Pose2DWithCov& p) {
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        cov(0, 0) = p.sigma_x * p.sigma_x;
        cov(1, 1) = p.sigma_y * p.sigma_y;
        cov(2, 2) = p.sigma_theta * p.sigma_theta;
        return cov;
    }
    inline Pose2DWithCov getEstimate() const {
        Pose2DWithCov result;
        result.pose.x = x_(0);
        result.pose.y = x_(1);
        result.pose.theta = x_(2);
    
        result.sigma_x = std::sqrt(P_(0, 0));
        result.sigma_y = std::sqrt(P_(1, 1));
        result.sigma_theta = std::sqrt(P_(2, 2));
    
        return result;
    }
    inline Eigen::Matrix3d getCovariance() const {
        return P_;
    }
    private:
        Eigen::Vector3d x_;   // State: [x, y, theta]
        Eigen::Matrix3d P_;   // Covariance
};
#include <stdio.h> 
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <Eigen/Dense>
#include <fstream>

#include "ina219.h"
#include "scan_match.h"
#include "lidar.h"
#include "ekf.h"
#include "globals.h"
#include "odometry.h"
#include "kalman.h"
#include "camera.h"
#include "servo.h"

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "spi_com.h"

//GLOBALS
int running = 1;
int frame_width = 640;
int frame_height = 480;
Box target_box = {};
LidarData lidar_data = { 0 };
std::mutex mutex;
Pose2DWithCov kalman_pose = { 0 };
Pose2DWithCov initial_pose = {{0.3, 1.2, 0}, 0.0001,0.0001,0.00001};
//-------

const float SHUNT_OHMS = 0.01;
const float MAX_EXPECTED_AMPS = 3.2;
const float Delay_ms = 1000.0;
const int USE_BATTERY = 1;

static const int SPI_CHANNEL = 1;


Eigen::Vector3d goal(1.42, 0.5, M_PI / 2);
Eigen::Vector3d ekf_pose;

enum ROBOT_STATE {
    TO_BLOCK,
    TO_BASE,
    REVERSE_AT_BASE,
    ROTATE_AT_BASE
};

ROBOT_STATE robot_state = TO_BLOCK;

std::ofstream kalman_log("kalman_log.txt");
std::ofstream odom_log("odometry_log.txt");
std::ofstream scanmatch_log("scanmatch_log.txt");

auto start_time = std::chrono::steady_clock::now();

double get_time_sec(std::chrono::steady_clock::time_point start) {
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - start;
    return elapsed.count();
}

void navigateToGoal(Eigen::Vector3d goal, Eigen::Vector3d current_pose, double& v_out, double& omega_out) {
    double Kp_ang = 2.0;
    double Kp_pos = 0.2;

    double dx = goal(0) - current_pose(0);
    double dy = goal(1) - current_pose(1);
    double distance = std::sqrt(dx*dx + dy*dy);
    double angle_to_goal = std::atan2(dy, dx);
    double angle_error = angle_to_goal - current_pose(2);
    double heading_error = goal(2) - current_pose(2);
    angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));
    heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));

    v_out = 0.0;
    omega_out = 0.0;

    if (std::abs(angle_error) > 0.1) {
        omega_out = Kp_ang * angle_error;
    } else if (distance > 0.02) {
        v_out = std::min(2.0, Kp_pos * distance);
        omega_out = Kp_ang * angle_error;
    } else if (std::abs(heading_error) > 0.025) {
        omega_out = Kp_ang * heading_error;
    } else {
        v_out = 0.0;
        omega_out = 0.0;
    }

    omega_out = std::max(std::min(omega_out, 0.5), -0.5);
    v_out = std::max(std::min(v_out, 2.0), -2.0);
}
void* check_stop_running(void *arg) {
    char temp;
    scanf("%c", &temp);

    std::lock_guard<std::mutex> lock(mutex);
    running = 0;
}

void* motor_thread(void *arg) {
    wiringPiSetup();
    wiringPiSPISetup(SPI_CHANNEL, 1000000);

    servo_start_pos();

    MotorDataType motor_data = {};

    long previous_encoder_m1 = -1, previous_encoder_m2 = -1; 
    double wheel_base = 0.14;
    double wheel_circumference = 0.03 * M_PI;
    double delta_time_s = 0.05;

    EKF ekf = EKF();
    ekf.init(initial_pose);
    
    while (running) {
        if(lidar_data.has_scanned_once) {
            double vel = 0, angle = 0;
            Eigen::Vector3d v;
            v << kalman_pose.pose.x, kalman_pose.pose.y, kalman_pose.pose.theta;

	    printf("Robot state %i\n", robot_state);
	    if (robot_state == TO_BASE) {
		goal << initial_pose.pose.x, initial_pose.pose.y, 0;
		navigateToGoal(goal, v, vel, angle);

		if (fabs(goal(0) - v(0)) < 0.07 && fabs(goal(1) - v(1)) < 0.07) {
		   robot_state = ROTATE_AT_BASE;
		   servo_up();
		}
	    }
	    if (robot_state == TO_BLOCK) {
	    	if(target_box.pixel_area > 0) {
                    double fov = 60.0f * M_PI / 180.0f;
                    double adjusted_angle = target_box.dst_from_center * fov / frame_width;
                    //printf("Angle %f\n", adjusted_angle * 180.0f / M_PI);
                    goal(2) = kalman_pose.pose.theta - adjusted_angle;
                    goal(0) = kalman_pose.pose.x + std::cos(goal(2)) * 0.5;
                    goal(1) = kalman_pose.pose.y + std::sin(goal(2)) * 0.5;

                    navigateToGoal(goal, v, vel, angle);
		    if (target_box.y < 90) {
		        servo_down();
		        robot_state = TO_BASE;
		    }
	    	}
	    }
	    if (robot_state == ROTATE_AT_BASE) {
		goal(2) = initial_pose.pose.theta;
                goal(0) = initial_pose.pose.x + std::cos(goal(2)) * 0.3;
        	goal(1) = initial_pose.pose.y + std::sin(goal(2)) * 0.3;
		navigateToGoal(goal, v, vel, angle);
		
		if (fabs(goal(0) - v(0)) < 0.07 && fabs(goal(1) - v(1)) < 0.07) {
		    robot_state = TO_BLOCK;
		}
	    }
	    
            double v_left  = vel + (angle * wheel_base / 2.0);
            double v_right = vel - (angle * wheel_base / 2.0);
            double left_rpm  = (v_left  / wheel_circumference) * 60.0;
            double right_rpm = (v_right / wheel_circumference) * 60.0;

            motor_data.Set_Speed_M1 = -left_rpm * 50;
            motor_data.Set_Speed_M2 = right_rpm * 50;

            Send_Read_Motor_Data(&motor_data);
                
            if (previous_encoder_m1 == -1 || previous_encoder_m2 == -1) {
                previous_encoder_m1 = motor_data.Encoder_M1;
                previous_encoder_m2 = motor_data.Encoder_M2;
            }

            Pose2DWithCov current_odom_res = computeOdometryDelta(motor_data.Encoder_M1, motor_data.Encoder_M2, previous_encoder_m1, previous_encoder_m2, 0.01);

            ekf.predict(current_odom_res);
	    double t = get_time_sec(start_time);
	    if (odom_log.is_open()) {
    	     odom_log << t << " "
             << current_odom_res.pose.x * 1000 << " "
             << current_odom_res.pose.y * 1000 << " "
             << current_odom_res.pose.theta << " "
             << current_odom_res.sigma_x << " 0 0 "
             << "0 " << current_odom_res.sigma_y << " 0 "
             << "0 0 " << current_odom_res.sigma_theta << "\n";
	    }

            previous_encoder_m1 = motor_data.Encoder_M1;
            previous_encoder_m2 = motor_data.Encoder_M2;


            if (lidar_data.has_scanned) {
                lidar_data.has_scanned = 0;

                ekf.update(lidar_data.current_lidar_scan.result);
                kalman_pose = ekf.getEstimate();
		t = get_time_sec(start_time);
                
                printf("Kalman:      %f, %f, %f\n", kalman_pose.pose.x, kalman_pose.pose.y, kalman_pose.pose.theta);
		printf("Scan match:      %f, %f, %f\n", lidar_data.current_lidar_scan.result.pose.x, lidar_data.current_lidar_scan.result.pose.y, lidar_data.current_lidar_scan.result.pose.theta);
                printf("Uncertainty: %f, %f, %f\n", kalman_pose.sigma_x,kalman_pose.sigma_y, kalman_pose.sigma_theta);     
            
		Pose2DWithCov scan_result = lidar_data.current_lidar_scan.result;
		
		if (scanmatch_log.is_open()) {
		    scanmatch_log << t << " "
		                  << scan_result.pose.x * 1000 << " "
		                  << scan_result.pose.y * 1000 << " "
		                  << scan_result.pose.theta << " "
		                  << scan_result.sigma_x << " 0 0 "
		                  << "0 " << scan_result.sigma_y << " 0 "
		                  << "0 0 " << scan_result.sigma_theta << "\n";
		}


		Eigen::Matrix3d P = ekf.getCovariance();
		if (kalman_log.is_open()) {
	    		kalman_log << t << " "
	               << kalman_pose.pose.x * 1000 << " "
	               << kalman_pose.pose.y * 1000 << " "
	               << kalman_pose.pose.theta << " "
	               << P(0,0) << " " << P(0,1) << " " << P(0,2) << " "
	               << P(1,0) << " " << P(1,1) << " " << P(1,2) << " "
               		<< P(2,0) << " " << P(2,1) << " " << P(2,2) << "\n";
		}
	    }
        }

        if (!lidar_data.is_scanning) {
            lidar_data.current_lidar_scan.pose = kalman_pose.pose;
            lidar_data.is_scanning = 1;
        }

        usleep(delta_time_s * 1e6);
    }
}

int main(void){
    lidar_data.current_lidar_scan.pose = initial_pose.pose;
    lidar_data.current_lidar_scan.result = initial_pose;
    kalman_pose = initial_pose;

    pthread_t stop_running_thread;
    pthread_create(&stop_running_thread, 0, check_stop_running, 0);

    //Init battery
    INA219 Ina(SHUNT_OHMS, MAX_EXPECTED_AMPS);
    Ina.configure(RANGE_16V, GAIN_8_320MV, ADC_12BIT, ADC_12BIT);

    pthread_t lidar_start_thread, lidar_listen_thread;

        pthread_create(&lidar_start_thread, 0, lidar_start_receiving, 0);
 
        // Start lidar
        lidar_running(1);

        while (!lidar_data.lidar_connected && running) {
            printf("Lidar not connected!\n");
            usleep(1.0f * 1e6);
        }

        // Start listening
        pthread_create(&lidar_listen_thread, 0, lidar_listen, 0);

    pthread_t m_thread;
    pthread_create(&m_thread, 0, motor_thread, 0);

    pthread_t m_camera_thread;
    pthread_create(&m_camera_thread, 0, camera_thread, 0);

    while(running){
        if (USE_BATTERY) {
            float current = Ina.current();
            float Volt = Ina.supply_voltage();

            if (Volt < 10.8)
                printf("WARNING: VOLTAGE BELOW 10.8");
            if (Volt < 10.5) {
                printf("Voltage is less than 10.5, stopping the program...\n");
                std::lock_guard<std::mutex> lock(mutex);
                running = 0;
            }
        }	
    }

    //servo_up();

    void *ret;
    pthread_join(lidar_listen_thread, &ret);

    lidar_running(0);
    lidar_shutdown();

    camera_release();

    return 0;
}
#include "camera.h"
#include "globals.h"

cv::VideoCapture cap;

void* camera_thread(void *arg) {
    cap = cv::VideoCapture(0);
    if (!cap.isOpened()) {
        printf("ERROR: Unable to open the camera\n");
        return 0;
    }

    int frame_width = 640;
    int frame_height = 480;
    int center_x = frame_width / 2;
    int center_y = frame_height / 2;
    std::vector<Box> found_boxes;

    cap.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
    cap.set(cv::CAP_PROP_FPS, 10);

    cv::namedWindow("Object Detection", cv::WINDOW_NORMAL);
    cv::resizeWindow("Object Detection", frame_width, frame_height);
    std::cout << "Camera started!" << std::endl;

    cv::Mat frame;
    while (running) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "ERROR: Unable to grab from the camera" << std::endl;
            continue;
        }

        // Convert to HSV and apply blue filter
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Adjust these values based on your lighting and camera
        cv::Scalar lower_blue(100, 50, 50);  // H, S, V
        cv::Scalar upper_blue(130, 255, 255);
        cv::inRange(hsv, lower_blue, upper_blue, mask);

        // Morphological operations to clean the mask
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        // Find contours on the mask directly
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        found_boxes.clear();

	for (const auto& contour : contours) {
	    double area = cv::contourArea(contour);
	    //if (area < 500) continue;
	
	    // Rotated rectangle around the contour
	    cv::RotatedRect rot_rect = cv::minAreaRect(contour);
	    float rect_area = rot_rect.size.width * rot_rect.size.height;
	
	    if (rect_area <= 0) continue;
	
	    int object_x = rot_rect.center.x;
	    int distance = center_x - object_x;
	
	    cv::Rect bounding_box = rot_rect.boundingRect();
	    found_boxes.push_back({bounding_box, area, distance, frame_height-(int)rot_rect.center.y});
	
	    // Draw the rotated rectangle
	    cv::Point2f points[4];
	    rot_rect.points(points);
	    for (int i = 0; i < 4; ++i)
	        cv::line(frame, points[i], points[(i+1)%4], cv::Scalar(0, 255, 0), 2);
	}

        Box largest_box = {};
        for (const auto& box : found_boxes) {
            if (box.pixel_area > largest_box.pixel_area) {
                largest_box = box;
            }
        }
        target_box = largest_box;

        if (largest_box.pixel_area != 0) {
            int object_x = largest_box.rect.x + largest_box.rect.width / 2;
            int object_y = largest_box.rect.y + largest_box.rect.height / 2;
            cv::circle(frame, cv::Point(object_x, object_y), 5, cv::Scalar(255, 0, 0), -1);
            printf("Distance from center: %i pixels\n", largest_box.dst_from_center);
        }

        cv::imshow("Object Detection", frame);

        if (cv::waitKey(5) == 'q') break;
    }

    return 0;
}
void camera_release() {
    cap.release();
    cv::destroyAllWindows();
}

#if 0

#include "camera.h"
#include "globals.h"

cv::VideoCapture cap;

void* camera_thread(void *arg) {
    cap = cv::VideoCapture(0);
    if (!cap.isOpened()) {
	printf("ERROR: Unable to open the camera\n");
	return 0;
    }

    int frame_width = 640;
    int frame_height = 480;
    int center_x = frame_width / 2;
    int center_y = frame_height / 2;
    std::vector<Box> found_boxes;

    cap.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
    cap.set(cv::CAP_PROP_FPS, 10);

    cv::namedWindow("Object Detection", cv::WINDOW_NORMAL);
    cv::resizeWindow("Object Detection", frame_width, frame_height);

    std::cout << "Camera started!" << std::endl;

    cv::Mat frame;
    while (running) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "ERROR: Unable to grab from the camera" << std::endl;
            continue;
        }

        cv::Mat hsv, mask, masked_frame;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Define HSV color range for object detection (e.g., green)
        cv::Scalar lower(100, 50, 50);
        cv::Scalar upper(130, 255, 255);

        cv::inRange(hsv, lower, upper, mask);
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);  // Remove small white noise
	cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel); // Fill small holes in detected objects
        cv::bitwise_and(frame, frame, masked_frame, mask);

        // Convert to grayscale
        cv::Mat gray, blurred, thresh, edges;
        cv::cvtColor(masked_frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0);
        cv::adaptiveThreshold(blurred, thresh, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2);

        cv::Canny(thresh, edges, 30, 100);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(edges, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        found_boxes.clear();
	for (size_t i = 0; i < contours.size(); i++) {
    	    if (hierarchy[i][3] != -1) continue;  // skip child contours

 	    const std::vector<cv::Point>& contour = contours[i];
            double area = cv::contourArea(contour);
            if (area < 500) continue;

            std::vector<cv::Point> hull;
            cv::convexHull(contour, hull);
            double hull_area = cv::contourArea(hull);
            if (hull_area == 0) continue;

            double solidity = area / hull_area;
            std::vector<cv::Point> approx;
            cv::approxPolyDP(contour, approx, 0.02 * cv::arcLength(contour, true), true);

            if (approx.size() >= 4 && solidity > 0.3) {
                cv::Rect rect = cv::boundingRect(approx);
                int object_x = rect.x + rect.width / 2;
                int distance = center_x - object_x;

                found_boxes.push_back({rect, area, distance});

                cv::rectangle(frame, rect, cv::Scalar(0, 255, 0), 2);
            }
        }

        Box largest_box = {};
        for (const auto& box : found_boxes) {
            if (box.pixel_area > largest_box.pixel_area) {
                largest_box = box;
            }
        }
	target_box = largest_box;
        if (largest_box.pixel_area != 0) {
            int object_x = largest_box.rect.x + largest_box.rect.width / 2;
            int object_y = largest_box.rect.y + largest_box.rect.height / 2;
            
            cv::circle(frame, cv::Point(object_x, object_y), 5, cv::Scalar(255, 0, 0), -1);

            printf("Distance from center: %i pixels\n", largest_box.dst_from_center);
        }

        cv::imshow("Object Detection", frame);

        if (cv::waitKey(5) == 'q') {
            break;
        }
    }

    return 0;
}

void camera_release() {
    cap.release();
    cv::destroyAllWindows();
}
#endif
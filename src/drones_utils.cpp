#include "lab3_drones/drones_utils.hpp"
#include <ros/ros.h>


// #define M_PI 3.141592653589793238462643383279502884197

struct CameraParams {
    double focal_length; // Focal length of the camera in meters
    double width;        // Image width in pixels
    double height;       // Image height in pixels
    double cx;           // Principal point x-coordinate (center of image)
    double cy;           // Principal point y-coordinate (center of image)
    double fov_x;        // Field of view in the x direction (radians)
    double fov_y;        // Field of view in the y direction (radians)
};

bool isGateInsideCamera(const cv::Point2f& projected_point, const CameraParams& camera) {
    // Check if the point is within the camera's image boundaries
    return ((projected_point.x >= 0) && (projected_point.x <= camera.width) &&
            (projected_point.y >= 0) && (projected_point.y <= camera.height));
}


tf2::Matrix3x3 getCameraRotation() {
    // Define the fixed rotation: Transform from drone frame to camera frame
    tf2::Matrix3x3 rotation_matrix;
    rotation_matrix.setValue( 0,  1,  0,  // Drone X -> Camera Z
                              0,  0, -1,  // Drone Y -> Camera X
                              1,  0,  0); // Drone Z -> Camera -Y

    // rotation_matrix.setValue( 0,  0,  1,  // Drone X -> Camera Z
    //                           1,  0, 0,  // Drone Y -> Camera X
    //                           0,  -1,  0); // Drone Z -> Camera -Y
    return rotation_matrix;
}

cv::Point2f projectGateToCamera(const geometry_msgs::Pose& gate_pose, const nav_msgs::Odometry& drone_odom, const CameraParams& camera) {
    // Extract drone's position
    double drone_x = drone_odom.pose.pose.position.x;
    double drone_y = drone_odom.pose.pose.position.y;
    double drone_z = drone_odom.pose.pose.position.z;

    // Extract drone's orientation (quaternion)
    double qx = drone_odom.pose.pose.orientation.x;
    double qy = drone_odom.pose.pose.orientation.y;
    double qz = drone_odom.pose.pose.orientation.z;
    double qw = drone_odom.pose.pose.orientation.w;

    // Create quaternion and transform matrix
    tf2::Quaternion drone_orientation(qx, qy, qz, qw);
    tf2::Matrix3x3 drone_orientation_mat(drone_orientation);
    tf2::Matrix3x3 camera_fixed_rotation = getCameraRotation();
    tf2::Matrix3x3 rotation_matrix = drone_orientation_mat * camera_fixed_rotation;

    // Gate's position in world frame
    tf2::Vector3 gate_world_pos(gate_pose.position.x, gate_pose.position.y, gate_pose.position.z);

    // Drone's position in world frame
    tf2::Vector3 drone_world_pos(drone_x, drone_y, drone_z);

    // Relative position of the gate in the world frame
    tf2::Vector3 gate_camera_frame = rotation_matrix.transpose()*gate_world_pos - (rotation_matrix.transpose()*drone_world_pos);

    // Transform the relative position into the camera (local drone) frame
    // tf2::Vector3 gate_camera_frame = rotation_matrix.transpose() * relative_pos - ;

    // Camera frame convention: assuming camera looks along the +Z axis
    double rel_gate_x = gate_camera_frame.x();
    double rel_gate_y = gate_camera_frame.y();
    double rel_gate_z = gate_camera_frame.z();

    // Check if gate is behind the camera
    if (rel_gate_z <= 0) {
        return cv::Point2f(-1, -1);  // Invalid point; gate is not visible
    }

    // Assuming camera is looking along the Z-axis (i.e., forward direction)
    // Apply the camera's intrinsic parameters to project the point
    double fx = camera.focal_length;  // Focal length in x-direction (camera focal length)
    double fy = fx * (camera.height / camera.width);  // Focal length in y-direction based on aspect ratio

    // Apply pinhole camera model projection
    double u = fx * (rel_gate_x / rel_gate_z) + camera.cx;
    double v = fy * (rel_gate_y / rel_gate_z) + camera.cy;

    return cv::Point2f(u, v);  // Projected point in 2D image plane
}

geometry_msgs::Quaternion RPYToQuat(double roll, double pitch, double yaw) {
    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    return quaternion;
}

Eigen::Matrix<double, 3, 3> quatToRMatrix(const geometry_msgs::Quaternion& quat) {
    Eigen::Quaterniond eigen_quat(quat.w, quat.x, quat.y, quat.z);
    return eigen_quat.toRotationMatrix();
}

Eigen::Matrix<double, 3, 3> RPYtoRMatrix(double roll, double pitch, double yaw) {
    Eigen::AngleAxis<double> rollAngle(roll, Eigen::Matrix<double, 1, 3>::UnitX());
    Eigen::AngleAxis<double> pitchAngle(pitch, Eigen::Matrix<double, 1, 3>::UnitY());
    Eigen::AngleAxis<double> yawAngle(yaw, Eigen::Matrix<double, 1, 3>::UnitZ());

    Eigen::Matrix<double, 3, 3> R;

    Eigen::Quaternion<double> q = yawAngle * rollAngle * pitchAngle;

    R = q.matrix();

    return (R);
}


double getYawFromQuaternion(const geometry_msgs::Quaternion& quat) {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    q.normalize();
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

void calculateMetrics(
    const std::vector<nav_msgs::Odometry>& gt_poses,
    const std::vector<geometry_msgs::PoseStamped>& goal_list_,
    const std::vector<geometry_msgs::Twist>& goal_vel_list_,
    std::string yaw_control,
    std::vector<int> objective_gates,
    std::vector<geometry_msgs::Pose> gates_)
{
    if (gt_poses.empty()) {
        ROS_WARN("Ground truth is empty. Cannot compute metrics.");
        return;
    }
    if (goal_list_.empty()) {
        ROS_WARN("Goal list is empty. Cannot compute metrics.");
        return;
    }

    std::vector<double> lowest_distances, angular_errors, velocity_errors;
    const double epsilon = 1e-6;

    ROS_INFO("Calculating Metrics (use_orientation: %s)...", yaw_control);
    ROS_INFO("Ground Truth Size: %lu", gt_poses.size());
    ROS_INFO("Goal List Size: %lu", goal_list_.size());

    CameraParams camera;
    camera.focal_length = 1.0; // 1 meter focal length
    camera.width = 1920;        // 1920 pixels wide
    camera.height = 1080;       // 1080 pixels high
    camera.cx = camera.width / 2;  // Principal point in the center of the image
    camera.cy = camera.height / 2;
    camera.fov_x = 90 * M_PI / 180.0;  // 90 degrees in radians
    camera.fov_y = 60 * M_PI / 180.0;  // 60 degrees in radians
    long int gate_visible = 0;
    long int valid_gates = 0;

    for (std::size_t i = 0; i < gt_poses.size(); i++) {
        double lowest_dist = std::numeric_limits<double>::max();
        std::size_t index = 0;

        // Find the closest trajectory point
        for (std::size_t j = 0; j < goal_list_.size(); j++) {
            double dist = sqrt(
                pow(gt_poses[i].pose.pose.position.x - goal_list_[j].pose.position.x, 2) +
                pow(gt_poses[i].pose.pose.position.y - goal_list_[j].pose.position.y, 2) +
                pow(gt_poses[i].pose.pose.position.z - goal_list_[j].pose.position.z, 2));

            if (dist < lowest_dist) {
                index = j;
                lowest_dist = dist;
            }
        }

        lowest_distances.push_back(lowest_dist);

        // Calculate velocity error
        double velocity_error = sqrt(
            pow(gt_poses[i].twist.twist.linear.x - goal_vel_list_[index].linear.x, 2) +
            pow(gt_poses[i].twist.twist.linear.y - goal_vel_list_[index].linear.y, 2) +
            pow(gt_poses[i].twist.twist.linear.z - goal_vel_list_[index].linear.z, 2));

        velocity_errors.push_back(velocity_error);

        // Calculate ground truth yaw in the global frame
        // double gt_yaw = atan2(gt_poses[i].twist.twist.linear.y, gt_poses[i].twist.twist.linear.x);
        double gt_yaw = tf2::getYaw(gt_poses[i].pose.pose.orientation);
        double goal_yaw = 0.0,goal_yaw_ori = 0.0, goal_yaw_no_ori = 0.0;
        static double last_valid_goal_yaw = 0.0;

        if (yaw_control == "no_control"){
            // if (index + 1 < goal_list_.size()) {
            //     goal_yaw = atan2(
            //         goal_list_[index + 1].pose.position.y - goal_list_[index].pose.position.y,
            //         goal_list_[index + 1].pose.position.x - goal_list_[index].pose.position.x);
            //     goal_yaw = normalizeAngle(goal_yaw);
            //     last_valid_goal_yaw = goal_yaw;

            // } else {
            //     goal_yaw = last_valid_goal_yaw;
            // }
            goal_yaw_no_ori = atan2(goal_vel_list_[index].linear.y, goal_vel_list_[index].linear.x);
            goal_yaw = goal_yaw_no_ori;
        }
        else if ((yaw_control == "yaw_2D_vel") || (yaw_control == "RPY_control") || (yaw_control == "next_gate")){
            goal_yaw_ori = tf2::getYaw(goal_list_[index].pose.orientation);


            // Dan valores clavados así que esto chill
            if(i == 0 || i == 100 || i == 500 || i == 1000 || i == 1500 || i == 2000){
                ROS_INFO("Goal Yaw orientation : %.2f degrees", goal_yaw_ori * 180 / M_PI);
                ROS_INFO("Goal Yaw velocities: %.2f degrees", goal_yaw_no_ori * 180 / M_PI);
            }

            goal_yaw = goal_yaw_ori;
        }
        else{
            ROS_ERROR("Invalid yaw contol mode: %s Aborting!", yaw_control);
            return;
        }        

        // Normalize yaws to the range [-π, π]
        gt_yaw = normalizeAngle(gt_yaw);
        goal_yaw = normalizeAngle(goal_yaw);

        // Calculate angular error
        double angular_error = fabs(normalizeAngle(gt_yaw - goal_yaw));
        angular_errors.push_back(angular_error);
        if(i == 0 || i == 100 || i == 500 || i == 1000 || i == 1500 || i == 2000){

        
            // Detailed log for debugging
            ROS_INFO("---------- Debugging No Orientation ----------");
            ROS_INFO("Ground Truth Index: %lu", i);
            ROS_INFO("Ground Truth Position: (%.3f, %.3f, %.3f)", 
                    gt_poses[i].pose.pose.position.x, 
                    gt_poses[i].pose.pose.position.y, 
                    gt_poses[i].pose.pose.position.z);
            ROS_INFO("Goal Position: (%.3f, %.3f, %.3f)", 
                    goal_list_[index].pose.position.x, 
                    goal_list_[index].pose.position.y, 
                    goal_list_[index].pose.position.z);
            ROS_INFO("Ground Truth Yaw: %.2f degrees", gt_yaw * 180 / M_PI);
            ROS_INFO("Goal Yaw: %.2f degrees", goal_yaw * 180 / M_PI);
            ROS_INFO("Angular Error: %.2f degrees", angular_error * 180 / M_PI);
            ROS_INFO("----------------------------------------------");
        }

        if (objective_gates[i] != -1) {
            cv::Point2f projected_point = projectGateToCamera(gates_[i], gt_poses[i], camera);

            // Check if the gate is inside the camera's field of view
            if (isGateInsideCamera(projected_point, camera)) {
                // std::cout << "Gate is inside the camera's FOV at: (" 
                //         << projected_point.x << ", " 
                //         << projected_point.y << ")\n";
                gate_visible++;
            } else {
                // std::cout << "Gate is outside the camera's FOV.\n";
            }

            valid_gates++;
            
        }
    }

    // Compute mean and standard deviation for each metric
    double mean_distance = std::accumulate(lowest_distances.begin(), lowest_distances.end(), 0.0) / lowest_distances.size();
    double std_dev_distance = sqrt(std::inner_product(lowest_distances.begin(), lowest_distances.end(), lowest_distances.begin(), 0.0) / lowest_distances.size() - mean_distance * mean_distance);

    double mean_angular = std::accumulate(angular_errors.begin(), angular_errors.end(), 0.0) / angular_errors.size();
    double std_dev_angular = sqrt(std::inner_product(angular_errors.begin(), angular_errors.end(), angular_errors.begin(), 0.0) / angular_errors.size() - mean_angular * mean_angular);

    double mean_velocity = std::accumulate(velocity_errors.begin(), velocity_errors.end(), 0.0) / velocity_errors.size();
    double std_dev_velocity = sqrt(std::inner_product(velocity_errors.begin(), velocity_errors.end(), velocity_errors.begin(), 0.0) / velocity_errors.size() - mean_velocity * mean_velocity);

    // Final metrics output
    ROS_INFO("Metrics Summary:");
    ROS_INFO("Distance Errors - Mean: %.3f, Std Dev: %.3f", mean_distance, std_dev_distance);
    ROS_INFO("Angular Errors - Mean: %.3f radians (%.2f degrees), Std Dev: %.3f radians (%.2f degrees)", 
             mean_angular, mean_angular * 180 / M_PI, 
             std_dev_angular, std_dev_angular * 180 / M_PI);
    ROS_INFO("Velocity Errors - Mean: %.3f, Std Dev: %.3f", mean_velocity, std_dev_velocity);
    ROS_INFO("Gate Visibles: %d ", gate_visible);
    ROS_INFO("Valid Gates: %d ", valid_gates);
    ROS_INFO("Gate Visibles Perc: %.3f ", (static_cast<float>(gate_visible) / static_cast<float>(valid_gates))*100);
}

double angularDistance(double angle1, double angle2) {
    double diff = angle2 - angle1;
    // Normalize the angle to the range [-pi, pi]
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return std::fabs(diff);
}

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

Eigen::Vector3d extractOrientationAsRPY(const geometry_msgs::PoseStamped& pose_stamped) {
    // Extract the quaternion from the PoseStamped message
    const auto& q = pose_stamped.pose.orientation;

    // Convert the ROS quaternion to a tf2::Quaternion
    tf2::Quaternion tf_quat(q.x, q.y, q.z, q.w);

    // Convert the quaternion to roll, pitch, and yaw
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    // Return as an Eigen::Vector3d
    return Eigen::Vector3d(roll, pitch, yaw);
}

// Function to normalize a vector
void normalizeVector(double& x, double& y, double& z) {
    double magnitude = std::sqrt(x * x + y * y + z * z);
    if (magnitude == 0) {
        throw std::invalid_argument("Zero magnitude vector cannot be normalized.");
    }
    x /= magnitude;
    y /= magnitude;
    z /= magnitude;
}

// Function to calculate quaternion from 3D directional vector
geometry_msgs::Quaternion calculateQuaternionFromVector(double x, double y, double z) {
    // Normalize the vector
    normalizeVector(x, y, z);

    // Assume the z-axis (0, 0, 1) is the reference direction
    tf2::Vector3 reference(0.0, 0.0, 1.0);
    tf2::Vector3 target(x, y, z);

    // Compute the rotation axis (cross product) and angle
    tf2::Vector3 rotationAxis = reference.cross(target);
    double rotationAngle = std::acos(reference.dot(target));

    if (rotationAxis.length() < 1e-6) {
        // If the rotationAxis is too small, it means the vectors are aligned
        return tf2::toMsg(tf2::Quaternion(0, 0, 0, 1)); // Identity quaternion
    }

    rotationAxis.normalize();

    // Create the quaternion representing the rotation
    tf2::Quaternion quaternion(rotationAxis, rotationAngle);

    // Convert to ROS geometry_msgs::Quaternion
    geometry_msgs::Quaternion ros_quaternion;
    ros_quaternion.x = quaternion.x();
    ros_quaternion.y = quaternion.y();
    ros_quaternion.z = quaternion.z();
    ros_quaternion.w = quaternion.w();

    return ros_quaternion;
}
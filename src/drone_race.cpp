#include "lab3_drones/drone_race.hpp"

using namespace std;

DroneRace::DroneRace(ros::NodeHandle nh) : nh_(nh),timer_started_(false)
{
    // Read from parameters the path for the targets,
    // otherwise use a default value.
    if (!nh_.getParam("targets_file_path", targets_file_path_))
    {
        ROS_WARN("There is no 'targets_file_path' parameter. Using default value.");
        targets_file_path_ = "/home/arob/catkin_ws/src/p04_arob_lab3_drones/data/gates.txt";

    }
    // Try to open the targets file.
    if (!readGates_(targets_file_path_))
    {
        ROS_ERROR("Could not read targets from file: %s", targets_file_path_.c_str());
        ros::shutdown();
        return;
    }
    drone_finished = false;
    current_goal_idx_ = 0;
    // This variable will control if we are in pose or cmd_vel control mode
    is_pose_control_ = true;

    pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/command/pose", 1000);
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    // create publisher for RVIZ markers
    pub_traj_markers_ =
        nh_.advertise<visualization_msgs::MarkerArray>("/trajectory_markers", 1000);
    pub_traj_vectors_ =
        nh_.advertise<visualization_msgs::MarkerArray>("/trajectory_vectors", 1000);
    pub_gate_markers_ =
        nh_.advertise<visualization_msgs::MarkerArray>("/gate_markers", 1000);

    pose_sub = nh_.subscribe("/ground_truth/state", 1000, &DroneRace::dronePoseLogger, this);

    ros::Duration sleeptime(1.0);
    sleeptime.sleep(); // Sleep for a moment before trying to draw

    drawGates_();
    //dronePoseLogger();
    generateTrajectory_();
    cmd_timer_ = nh_.createTimer(ros::Duration(0.01), &DroneRace::commandTimerCallback_, this);
    ROS_INFO("DroneRace initialized");
}


bool DroneRace::readGates_(string file_name) {
    //Open the file
    ifstream input_file;
    input_file.open(file_name, ifstream::in);
    if (!input_file) {
        cerr << "Error opening the file." << endl;
        return false;
    }
    gates_.clear();

    geometry_msgs::Pose temp_pose;
    double yaw = 0;
    std::string line;
    while (std::getline(input_file, line))
    {
        std::istringstream iss(line);
        iss >> temp_pose.position.x;
        iss >> temp_pose.position.y;
        iss >> temp_pose.position.z;
        iss >> yaw;
        temp_pose.orientation = RPYToQuat_(0, 0, yaw);
        gates_.push_back(temp_pose);
    }

    // Close the file
    input_file.close();
    return true;
}

void DroneRace::commandTimerCallback_(const ros::TimerEvent& event) {

    if (current_goal_idx_ >= goal_list_.size()) {
        ROS_INFO("Reached the end of the trajectory");

        // Stop the timer and calculate the elapsed time
        if (timer_started_) {
            ros::Duration elapsed_time = ros::Time::now() - start_time_;
            ROS_INFO("Trajectory execution took %f seconds", elapsed_time.toSec());
            timer_started_ = false; // Reset the timer for future runs
            drone_finished = true;

            // Calculate Metrics function
            calculateMetrics();
        }

        ros::shutdown(); // End the program
        return;
    }

    // Start the timer on the first command
    if (!timer_started_) {
        start_time_ = ros::Time::now();
        timer_started_ = true;
    }

    // Publish either pose or velocity commands based on control mode
    if (is_pose_control_) {
        pub_goal_.publish(goal_list_[current_goal_idx_]);
    } else {
        pub_cmd_vel_.publish(goal_vel_list_[current_goal_idx_]);
    }
    current_goal_idx_++;
}


void DroneRace::dronePoseLogger(const nav_msgs::Odometry& odom_msg){
    if (!drone_finished) {
        gt_poses.push_back(odom_msg);
    }
}


void DroneRace::calculateMetrics() {

    if (gt_poses.empty()) {
        ROS_WARN("Ground truth is empty. Cannot compute metrics.");
        return;
    }
    if (goal_list_.empty()) {
        ROS_WARN("Goal list truth is empty");
        return;
    }

    std::vector<double> lowest_distances, angular_errors, velocity_errors;
    double lowest_dist = 100000; 
    int index = 0;

    // Iterate over gt_poses and goal_list_ assuming they are aligned
    for (int i = 0; i < gt_poses.size(); i++) {
        for(int j = 0; j < goal_list_.size() ; j++){
        
            // Compute Euclidean distance
            double dist = sqrt(pow(gt_poses[i].pose.pose.position.x - goal_list_[j].pose.position.x, 2) +
                               pow(gt_poses[i].pose.pose.position.y - goal_list_[j].pose.position.y, 2) +
                               pow(gt_poses[i].pose.pose.position.z - goal_list_[j].pose.position.z, 2));

            // Save lowest index and dist
            if (dist < lowest_dist){
                index = j; 
                lowest_dist = dist; 
            }
        }

        // Saving velocity and distance at same point.
        lowest_distances.push_back(lowest_dist);

        // Compute velocity error
        double velocity_error = sqrt(pow(gt_poses[i].twist.twist.linear.x - goal_vel_list_[index].linear.x, 2) +
                                     pow(gt_poses[i].twist.twist.linear.y - goal_vel_list_[index].linear.y, 2) +
                                     pow(gt_poses[i].twist.twist.linear.z - goal_vel_list_[index].linear.z, 2));

        velocity_errors.push_back(velocity_error);
        
        // Compute angular error (yaw difference)
        double gt_yaw = atan2(gt_poses[i].twist.twist.linear.y, gt_poses[i].twist.twist.linear.x);
        double goal_yaw = atan2(goal_vel_list_[index].linear.y, goal_vel_list_[index].linear.x);
        double angular_error = gt_yaw - goal_yaw;
        angular_error = atan2(sin(angular_error), cos(angular_error));
        angular_errors.push_back(angular_error);
    }

    if (lowest_distances.empty() || angular_errors.empty() || velocity_errors.empty()) {
        ROS_WARN("Error vectors are empty. Metrics cannot be computed.");
        return;
    }

    double sum_distance = 0.0, sum_distance_squared = 0.0;
    double sum_angular = 0.0, sum_angular_squared = 0.0;
    double sum_velocity = 0.0, sum_velocity_squared = 0.0;


    for(int i = 0; i < lowest_distances.size(); i++){
        sum_distance += lowest_distances[i];
        sum_distance_squared += lowest_distances[i] * lowest_distances[i];
        sum_angular += angular_errors[i];
        sum_angular_squared += angular_errors[i] * angular_errors[i];
        sum_velocity += velocity_errors[i];
        sum_velocity_squared += velocity_errors[i] * velocity_errors[i];
    }

    double mean_distance = sum_distance / lowest_distances.size();
    double std_dev_distance = sqrt((sum_distance_squared / lowest_distances.size()) - (mean_distance * mean_distance));
    double mean_angular = sum_angular / angular_errors.size();
    double std_dev_angular = sqrt((sum_angular_squared / angular_errors.size()) - (mean_angular * mean_angular));
    double mean_velocity = sum_velocity / velocity_errors.size();
    double std_dev_velocity = sqrt((sum_velocity_squared / velocity_errors.size()) - (mean_velocity * mean_velocity));

    // Print metrics
    ROS_INFO("Distance Errors - Mean: %f, Std Dev: %f", mean_distance, std_dev_distance);
    ROS_INFO("Angular Errors - Mean: %f, Std Dev: %f", mean_angular, std_dev_angular);
    ROS_INFO("Velocity Errors - Mean: %f, Std Dev: %f", mean_velocity, std_dev_velocity);
}





void DroneRace::generateTrajectory_() {
    //constants
    const int dimension = 3; //we only compute the trajectory in x, y and z
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP; //POSITION, VELOCITY, ACCELERATION, JERK, SNAP
    double max_vel = 3;
    double min_vel = 0.25;
    double vel = 0.85;
    mav_trajectory_generation::Vertex::Vector vertices;
    
    // INCLUDE YOUR CODE HERE
    mav_trajectory_generation::Vertex start(dimension), end(dimension);
    start.makeStartOrEnd(Eigen::Vector3d(0,0,1), derivative_to_optimize);
    vertices.push_back(start);
    // Provide the time constraints on the vertices
    Eigen::Matrix<double, 3, 1> v_gate;
    Eigen::Matrix<double, 3, 1> vel_gate_frame;


    for (size_t i = 0; i < gates_.size(); ++i) {
        const auto& gate = gates_[i];
        auto next_gate = gates_[(i+1) % gates_.size()];

        Eigen::Matrix<double, 3, 3> orientation_next_gate = quatToRMatrix_(next_gate.orientation);
        Eigen::Matrix<double, 3, 3> rotate_gate = quatToRMatrix_(gate.orientation);
        Eigen::Matrix<double, 3, 1> pos_gate(gate.position.x, gate.position.y, gate.position.z);
        Eigen::Matrix<double, 3, 1> pos_next_gate(next_gate.position.x, next_gate.position.y, next_gate.position.z);
        // maybe pos_gate
        Eigen::Matrix<double, 3, 1> distance_vec = (pos_next_gate - pos_gate);

        vel = std::max(min_vel, std::min(distance_vec.norm()*0.95, max_vel));

        vel_gate_frame << vel, 0.0, 0.0;
        v_gate = rotate_gate * vel_gate_frame;

        mav_trajectory_generation::Vertex middle(dimension);
        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(gate.position.x,gate.position.y,gate.position.z));
        middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(v_gate[0],v_gate[1],v_gate[2]));

        vertices.push_back(middle);
    }

    end.makeStartOrEnd(Eigen::Vector3d(0,0,1), derivative_to_optimize);
    vertices.push_back(end);
    std::vector<double> segment_times;
    // INCLUDE YOUR CODE HERE
    const double v_max = 3;
    const double a_max = 3;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);

    // Solve the optimization problem
    const int N = 10; //Degree of the polynomial, even number at least two times the highest derivative
    mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    opt.solveLinear();

    //Obtain the trajectory
    trajectory_.clear();
    opt.getTrajectory(&trajectory_);

    //Sample the trajectory (to obtain positions, velocities, etc.)
    mav_msgs::EigenTrajectoryPoint::Vector states;
    double sampling_interval = 0.01; //How much time between intermediate points
    bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory_, sampling_interval, &states);
    
    if (!success) {
        ROS_WARN("Trajectory sampling failed.");
        return;
    }

    cout << "Trajectory time = " << trajectory_.getMaxTime() << endl;
    cout << "Number of states = " << states.size() << endl;
    cout << "Position (world frame) " << 3 << " X = " << states[2].position_W[0] << endl;
    cout << "Velocity (world frame) " << 3 << " X = " << states[2].velocity_W[0] << endl;

    // Default Visualization
    visualization_msgs::MarkerArray markers;
    double distance = 0.5; // Distance by which to seperate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";
    mav_trajectory_generation::drawMavTrajectory(trajectory_, distance, frame_id, &markers);
    pub_traj_vectors_.publish(markers);

    //AROB visualization
    drawTrajectoryMarkers_();
    ROS_INFO("Generating trajectory commands.");

    //Including in the list the PoseStamped and the Twist Messages
    for (const auto& state : states) {
        goal_.header.frame_id = "world";
        goal_.header.stamp = ros::Time::now();
        goal_.pose.position.x = state.position_W.x();
        goal_.pose.position.y = state.position_W.y();
        goal_.pose.position.z = state.position_W.z();
        goal_list_.push_back(goal_); 

        goal_vel_.linear.x = state.velocity_W.x();
        goal_vel_.linear.y = state.velocity_W.y();
        goal_vel_.linear.z = state.velocity_W.z();
        goal_vel_list_.push_back(goal_vel_);
    }



}


Eigen::Matrix<double, 3, 3> DroneRace::RPYtoRMatrix_(double roll, double pitch, double yaw) {
    Eigen::AngleAxis<double> rollAngle(roll, Eigen::Matrix<double, 1, 3>::UnitX());
    Eigen::AngleAxis<double> pitchAngle(pitch, Eigen::Matrix<double, 1, 3>::UnitY());
    Eigen::AngleAxis<double> yawAngle(yaw, Eigen::Matrix<double, 1, 3>::UnitZ());

    Eigen::Matrix<double, 3, 3> R;

    Eigen::Quaternion<double> q = yawAngle * rollAngle * pitchAngle;

    R = q.matrix();

    return (R);
}

Eigen::Matrix<double, 3, 3> DroneRace::quatToRMatrix_(geometry_msgs::Quaternion q) {
    double roll, pitch, yaw;
    tf2::Quaternion quat_tf;
    tf2::fromMsg(q, quat_tf);
    Eigen::Matrix<double, 3, 3> mat_res;
    tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

    return RPYtoRMatrix_(roll, pitch, yaw);
}

geometry_msgs::Quaternion DroneRace::RPYToQuat_(double roll, double pitch, double yaw) {
    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    return quaternion;
}

void DroneRace::drawGates_() {
    int id = 0;
    for (geometry_msgs::Pose gate : gates_) {
        drawGateMarkers_(gate,id);
    }
}

void DroneRace::drawGateMarkers_(geometry_msgs::Pose gate, int &id){
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker line_marker;
    //std::vector<visualization_msgs::Marker> line_marker_vector;
    
    Eigen::Matrix<double, 3, 3> rotate_gate = quatToRMatrix_(gate.orientation);
    Eigen::Matrix<double, 3, 1> pos_gate(gate.position.x, gate.position.y, gate.position.z);

    marker.header.frame_id = "world";  // Change this frame_id according to your setup
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "corner";
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = ros::Duration();

    line_marker.header.frame_id = "world";  // Change this frame_id according to your setup
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "line";
    line_marker.id = id;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.scale.x = 0.05;  // Line width
    line_marker.pose.orientation.w = 1.0;
    line_marker.lifetime = ros::Duration();

    // Set the color (green in this case)
    line_marker.color.r = 0.0;
    line_marker.color.g = 1.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;

    float gate_size = 0.75;

    //Generate the gate corners and edges
    Eigen::Matrix<double, 3, 1> move_gate;
    move_gate << 0.0, gate_size, gate_size;
    Eigen::Matrix<double, 3, 1> position2 = pos_gate + rotate_gate * move_gate;
    marker.pose.position.x = position2(0);
    marker.pose.position.y = position2(1);
    marker.pose.position.z = position2(2);
    // Change left gate vertex color to yellow
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    // Change shape to sphere radius 25cm
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.id = id + 1;
    line_marker.points.push_back(marker.pose.position);
    marker_array.markers.push_back(marker);

    move_gate << 0.0, -gate_size, gate_size;
    Eigen::Matrix<double, 3, 1> position = pos_gate + rotate_gate * move_gate;
    marker.pose.position.x = position(0);
    marker.pose.position.y = position(1);
    marker.pose.position.z = position(2);
    // Change left gate vertex color to yellow
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    // Change shape to sphere radius 25cm
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.id = id + 2;
    line_marker.points.push_back(marker.pose.position);
    marker_array.markers.push_back(marker);

    move_gate << 0.0, -gate_size, -gate_size;
    position = pos_gate + rotate_gate * move_gate;
    marker.pose.position.x = position(0);
    marker.pose.position.y = position(1);
    marker.pose.position.z = position(2);
    marker.id = id + 3;
    line_marker.points.push_back(marker.pose.position);
    marker_array.markers.push_back(marker);

    move_gate << 0.0, gate_size, -gate_size;
    position = pos_gate + rotate_gate * move_gate;
    marker.pose.position.x = position(0);
    marker.pose.position.y = position(1);
    marker.pose.position.z = position(2);
    // Change right gate vertex color back to red
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    // Change right gate vertex color back to cube
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.id = id + 4;
    marker_array.markers.push_back(marker);
    line_marker.points.push_back(marker.pose.position);

    marker.pose.position.x = position2(0);
    marker.pose.position.y = position2(1);
    marker.pose.position.z = position2(2);
    line_marker.points.push_back(marker.pose.position);
    id+=5;
    marker_array.markers.push_back(line_marker);
    pub_gate_markers_.publish(marker_array);
}

void DroneRace::drawGoalMarker_(mav_trajectory_generation::Vertex goal){
    Eigen::VectorXd pos;
    goal.getConstraint(mav_trajectory_generation::derivative_order::POSITION,&pos);
    visualization_msgs::Marker marker_aux;
    marker_aux.header.frame_id = "world";
    marker_aux.header.stamp = ros::Time(0);
    marker_aux.id = id_marker;
    id_marker++;
    marker_aux.ns = "point";
    marker_aux.type = visualization_msgs::Marker::CUBE;
    marker_aux.pose.position.x = pos(0);
    marker_aux.pose.position.y = pos(1);
    marker_aux.pose.position.z = pos(2);
    marker_aux.pose.orientation.x = 0;
    marker_aux.pose.orientation.y = 0;
    marker_aux.pose.orientation.z = 0;
    marker_aux.pose.orientation.w = 1;
    marker_aux.scale.x = 0.1;
    marker_aux.scale.y = 0.1;
    marker_aux.scale.z = 0.1;
    marker_aux.color.r = 1.0f;
    marker_aux.color.g = 0.0f;
    marker_aux.color.b = 0.0f;
    marker_aux.color.a = 1.0;
    marker_aux.lifetime = ros::Duration();
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(marker_aux);
    pub_traj_markers_.publish(marker_array);
}

void DroneRace::drawTrajectoryMarkers_(){
    visualization_msgs::MarkerArray markers;
    int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
    double sampling_time = 0.1;
    mav_msgs::EigenTrajectoryPoint::Vector states;
    double sampling_interval = 0.1;
    bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory_, sampling_interval, &states);   
    for(int i=0; i< states.size(); i++) {
        visualization_msgs::Marker marker_aux;
        marker_aux.header.frame_id = "world";
        //marker_aux.header.stamp = ros::Time::now();
        marker_aux.header.stamp = ros::Time(0);
        marker_aux.id = 1000+i;
        marker_aux.ns = "point";
        marker_aux.type = visualization_msgs::Marker::CUBE;
        marker_aux.pose.position.x = states[i].position_W[0] ;
        marker_aux.pose.position.y = states[i].position_W[1] ;
        marker_aux.pose.position.z = states[i].position_W[2] ;
        marker_aux.pose.orientation.x = 0;
        marker_aux.pose.orientation.y = 0;
        marker_aux.pose.orientation.z = 0;
        marker_aux.pose.orientation.w = 1;
        marker_aux.scale.x = 0.03;
        marker_aux.scale.y = 0.03;
        marker_aux.scale.z = 0.03;
        marker_aux.color.r = 0.0f;
        marker_aux.color.g = 0.0f;
        marker_aux.color.b = 1.0f;
        marker_aux.color.a = 1.0;
        marker_aux.lifetime = ros::Duration();
        markers.markers.push_back(marker_aux);
    }
    pub_traj_markers_.publish(markers);
}


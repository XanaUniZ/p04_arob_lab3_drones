#include "lab3_drones/drone_race.hpp"
#include "lab3_drones/drawing.hpp"
#include "lab3_drones/drones_utils.hpp"

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

    use_orientation = true;

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

    //ros::Rate loop_rate(100); // 100 Hz en lugar de 50 Hz

    drawGates(gates_, pub_gate_markers_);
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
        temp_pose.orientation = RPYToQuat(0, 0, yaw);
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
            calculateMetrics(gt_poses, goal_list_, goal_vel_list_,use_orientation);
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

void DroneRace::generateTrajectory_() {
    //constants
    const int dimension = 3; //we only compute the trajectory in x, y and z
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP; //POSITION, VELOCITY, ACCELERATION, JERK, SNAP
    double max_vel = 2.5;
    double min_vel = 0.25;
    double vel = 0.55;
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

        Eigen::Matrix<double, 3, 3> orientation_next_gate = quatToRMatrix(next_gate.orientation);
        Eigen::Matrix<double, 3, 3> rotate_gate = quatToRMatrix(gate.orientation);
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
    const double v_max = 2.5;
    const double a_max = 2.5;
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
    drawTrajectoryMarkers(trajectory_, pub_traj_markers_);
    ROS_INFO("Generating trajectorys commands.");

    // Including in the list the PoseStamped and the Twist Messages

    if(!use_orientation){
        for (const auto& state : states) {
            goal_.header.frame_id = "world";
            goal_.header.stamp = ros::Time::now();
            goal_.pose.position.x = state.position_W.x();
            goal_.pose.position.y = state.position_W.y();
            goal_.pose.position.z = state.position_W.z();
            goal_.pose.orientation.y = 0.;
            goal_.pose.orientation.x = 0.;
            goal_.pose.orientation.z = 0.;
            goal_.pose.orientation.w = 1.;
            goal_list_.push_back(goal_); 

            goal_vel_.linear.x = state.velocity_W.x();
            goal_vel_.linear.y = state.velocity_W.y();
            goal_vel_.linear.z = state.velocity_W.z();
            goal_vel_list_.push_back(goal_vel_);
        }
    }
    else{
        for (size_t i = 0; i < states.size(); ++i) {
            // Crear el PoseStamped en el frame 'world' directamente
            goal_.header.frame_id = "world";
            goal_.header.stamp = ros::Time::now();

            // Asignar la posición de ground truth directamente desde states
            goal_.pose.position.x = states[i].position_W.x();
            goal_.pose.position.y = states[i].position_W.y();
            goal_.pose.position.z = states[i].position_W.z();

            // Calcular el yaw desde la velocidad en el frame 'world'
            // double yaw_from_vel = atan2(states[i].velocity_W.y(), states[i].velocity_W.x());
            const double epsilon = 1e-6;
            if (fabs(states[i].velocity_W.x()) > epsilon || fabs(states[i].velocity_W.y()) > epsilon) {
                double yaw_from_vel = atan2(states[i].velocity_W.y(), states[i].velocity_W.x());
                goal_.pose.orientation = RPYToQuat(0, 0, yaw_from_vel);
            } else {
                ROS_WARN("Velocity near zero at index %lu, setting goal_yaw to last valid yaw", i);
                goal_.pose.orientation = goal_list_.empty() ? RPYToQuat(0, 0, 0) : goal_list_.back().pose.orientation;
            }
            goal_list_.push_back(goal_);

            // Asignar la orientación calculada al goal_
            //goal_.pose.orientation = RPYToQuat(0, 0, yaw_from_vel);

            // Añadir el goal a la lista
            //goal_list_.push_back(goal_);

            // Asignar las velocidades en el frame 'world'
            goal_vel_.linear.x = states[i].velocity_W.x();
            goal_vel_.linear.y = states[i].velocity_W.y();
            goal_vel_.linear.z = states[i].velocity_W.z();
            goal_vel_list_.push_back(goal_vel_);
        }
    }
}

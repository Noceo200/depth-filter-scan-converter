#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <fstream>
#include <iostream> 
#include "rosgraph_msgs/msg/clock.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <cmath>
#include <Eigen/Dense>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

double TimeToDouble(builtin_interfaces::msg::Time& stamp);
bool consider_val(int current_ind, int start_ind, int end_ind);
int angle_to_index(double alpha, int resolution);
int remap_scan_index(int prev_ind, double prev_angle_start, double prev_angle_end, double prev_reso, double new_angle_start, double new_angle_end, double new_reso);
double sawtooth(double x, double period);
geometry_msgs::msg::Vector3 quaternion_to_euler3D(geometry_msgs::msg::Quaternion quat);
Eigen::MatrixXd get_4Dmatrix_from_transform(geometry_msgs::msg::TransformStamped transf);
Eigen::MatrixXd rot_matrix_from_euler(geometry_msgs::msg::Vector3 euler_angles);

class PointCloudToLaserScanNode : public rclcpp::Node
{
public:
    PointCloudToLaserScanNode() : Node("depth_to_scan_node")
    {
        initialize_params();
        refresh_params();
        if(debug){
            debug_params();
        }

        //QOS:
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

        //clock
        clock = this->get_clock();
        if(use_sim_time){
            clock_subscription_ = this->create_subscription<rosgraph_msgs::msg::Clock>("/clock", sensor_qos, std::bind(&PointCloudToLaserScanNode::ClockCallback, this, std::placeholders::_1));
        }

        //Tfs listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize subscriber to PointCloud2
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_in, sensor_qos, std::bind(&PointCloudToLaserScanNode::pointCloudCallback, this, std::placeholders::_1));

        // Initialize publisher for LaserScan
        if(publish_scan){
            laser_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_out_scan, default_qos);
        }

        // Initialize publisher for filtered point cloud
        if(publish_cloud){
            filtered_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_point_cloud", default_qos);
        }

        // Start loop
        if(publish_cloud || publish_scan){
            timer_ = create_wall_timer(std::chrono::milliseconds(int(1000/rate)), std::bind(&PointCloudToLaserScanNode::DepthFilterToScan, this));
        }
        else{
            std::stringstream debug_ss;
            debug_ss << "\nWARN: It seems that the node is configured to publish nothing. Check 'publish_scan' and 'publish_cloud' parameters." << std::endl;
            std::string debug_msg = debug_ss.str();
            write_debug(debug_file_path, debug_msg);
            RCLCPP_INFO(this->get_logger(), "%s", debug_msg.c_str());
        }
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::stringstream debug_ss;

        mutex_points_cloud.lock();
        raw_msg = msg;
        mutex_points_cloud.unlock();

        debug_ss << "\n\n" << "Data received... (timestamp: " << std::to_string(TimeToDouble(raw_msg->header.stamp)) <<" s)"<<" (node time: " << (this->now()).nanoseconds() << "ns)" << std::endl;

        //debug
        if(debug){
            std::string debug_msg = debug_ss.str();
            write_debug(debug_file_path, debug_msg);
        }
    }

    void DepthFilterToScan(){
        if(raw_msg != nullptr){
            std::stringstream debug_ss;
            
            // Copy data
            mutex_points_cloud.lock();
            sensor_msgs::msg::PointCloud2::SharedPtr msg(new sensor_msgs::msg::PointCloud2(*raw_msg));
            mutex_points_cloud.unlock();

            //convert PointCloud2 to points pcl object
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*msg, pcl_pc2);
            pcl::PointCloud<pcl::PointXYZ>::Ptr raw_points(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(pcl_pc2, *raw_points);

            //common initialisation
            int nb_points = raw_points->points.size();
            bool cloud_ready = false;
            bool scan_ready = false;
            cloud_ready = update_transform(transf_world_cam,ref_frame,msg->header.frame_id); //needed even if we don't publish the clouds
            if(publish_scan){
                scan_ready = update_transform(transf_laser_cam,out_frame,msg->header.frame_id);
            }

            if(cloud_ready){
                //filter initialisation
                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points(new pcl::PointCloud<pcl::PointXYZ>);
                Eigen::MatrixXd Pass_matrix_world_cam;
                Pass_matrix_world_cam = get_4Dmatrix_from_transform(transf_world_cam); //needed anyway by both feature

                //scan initialisation
                int scan_reso = static_cast<int>(round(h_fov/h_angle_increment));
                int circle_reso = static_cast<int>(round(scan_reso*(2*M_PI/h_fov)));
                int start_index = angle_to_index(-h_fov/2, circle_reso); //start index acording to fov on a 360 degree scan with circle_reso values, we take 360 so that we can throw points that are somewhere else on the circle
                int end_index = angle_to_index(h_fov/2, circle_reso);
                sensor_msgs::msg::LaserScan laser_scan_msg = new_clean_scan(); // To convert PointCloud2 to LaserScan
                Eigen::MatrixXd Pass_matrix_laser_cam;
                if(scan_ready){Pass_matrix_laser_cam = get_4Dmatrix_from_transform(transf_laser_cam);}

                //debug initialisation
                int nb_inbound_points = 0;
                int nb_cliff_points = 0;
                int added_as_obstacles = 0;
                int nb_thrown_points = 0;

                debug_ss << "\nStarting process for "<< nb_points << " points (timestamp: "<< std::to_string(TimeToDouble(msg->header.stamp)) <<" s)" << " (node time: " << (this->now()).nanoseconds() << "ns)" << std::endl;
                int t0;
                int tf;
                t0 = (this->now()).nanoseconds();

                // Iterate over each point in the PointCloud and fill filtered_points and laserscan
                for (int i =0; i<nb_points; i++)
                {
                    auto point = raw_points->points[i];
                    // Access the coordinates of the point in its frame, homogenous coordinates
                    Eigen::MatrixXd P_parent = Eigen::MatrixXd::Identity(4, 1); //position in parent frame
                    P_parent(0,0) = point.x;
                    P_parent(1,0) = point.y;
                    P_parent(2,0) = point.z;
                    //transform it to have the coordinate in the ref_frame, homogenous coordinates
                    Eigen::MatrixXd P_world = Pass_matrix_world_cam*P_parent;
                    double x = P_world(0,0);
                    double y = P_world(1,0);
                    double z = P_world(2,0);

                    // Process the point here...
                    double h = height_offset + z; //height in world
                    double angle = atan2(y,x); //angle from camera x axis
                    //debug_ss << "\nDEBUG_SPECIAL: " << "angle = " << angle << std::endl;
                    //debug_ss << "\nDEBUG_SPECIAL: " << "circle_reso = " << circle_reso << std::endl;
                    int ind_circle = angle_to_index(angle,circle_reso); //index where it should be in a 360 degree laserscan list of circle_reso values
                    float dist = sqrt(pow(x,2)+pow(y,2)); //planar from camera center

                    bool in_bounds = (min_height <= h) && (max_height >= h) && consider_val(ind_circle, start_index, end_index) && (range_min <= dist) && (range_max >= dist);
                    bool is_cliff = false;
                    /*if(cliff_detect){
                        is_cliff = h <= -cliff_height;
                    }*/ //MDP

                    if(in_bounds || is_cliff){
                        int real_ind = remap_scan_index(ind_circle, 0.0, 2*M_PI, circle_reso, -h_fov/2, h_fov/2, scan_reso); //we want the index for a list that represent values from -h_fov/2 values to h_fov/2 values. because the laserscan message is configured like this
                        //debug_ss << "\nDEBUG_SPECIAL: " << "circle_ind = " << ind_circle << " real_ind = " << real_ind << std::endl;
                        if(real_ind>0 && real_ind<laser_scan_msg.ranges.size()){
                            if(dist<laser_scan_msg.ranges[real_ind]){
                                laser_scan_msg.ranges[real_ind] = dist;
                            }
                            added_as_obstacles += 1;
                            laser_scan_msg.intensities[real_ind] = 0.0;
                        }
                        filtered_points->push_back(point); //for filtered point publishing
                        //other cases should never happen as we selected the points that should be in this laser scan already with 'in_bounds', but we add the condition just in case extremities indexes cause problems.
                    }

                    //debug
                    if(in_bounds){
                        nb_inbound_points += 1;
                    }
                    else if(is_cliff){
                        nb_cliff_points += 1;
                    }
                    else{
                        nb_thrown_points += 1;
                    }

                }

                // Convert the filtered point cloud to a ROS message
                sensor_msgs::msg::PointCloud2 filtered_msg;
                pcl::toROSMsg(*filtered_points, filtered_msg);
                filtered_msg.header = raw_msg->header; // Set the header

                //put clock
                if(use_sim_time){ //if use_sim_time = true
                    laser_scan_msg.header.stamp = simu_timestamp;
                    filtered_msg.header.stamp = simu_timestamp;
                }
                else{
                    laser_scan_msg.header.stamp = clock->now();
                    filtered_msg.header.stamp = clock->now();
                }

                //Publish filtered cloud points
                filtered_points_publisher_->publish(filtered_msg);

                // Publish the LaserScan message
                laser_scan_publisher_->publish(laser_scan_msg);

                //debug data
                int non_zero_vals = 0;
                int total_vals = laser_scan_msg.ranges.size();
                if(debug){
                    for(int i=0; i < total_vals; i++){
                        if(laser_scan_msg.ranges[i] < INFINITY){
                            non_zero_vals ++;
                        }
                    }
                }

                //debug
                debug_ss << "    |\n    |\n    V"
                        << "\nWe keep points in these bounds: "
                        << "\n  - Angle (horizontal fov): [" << -h_fov/2 << "," << h_fov/2 << "] rad"
                        << "\n  - Height: [" << min_height << "," << max_height << "] m"
                        << "\n  - Or hole with height (if cliff_detect = true): < " << -cliff_height << " m"
                        << "\n  - Planar Distance: " << range_min << "," << range_max << "] m"
                        << "\nPoints results: "
                        << "\n   Points in bounds: " << nb_inbound_points
                        << "\n   Holes: " << nb_cliff_points
                        << "\n   Total Obstacles: " << added_as_obstacles
                        << "\n   Ignored points: " << nb_thrown_points;

                debug_ss << "    |\n    |\n    V"
                        << "\nFiltered points cloud published" << " (node time: " << (this->now()).nanoseconds() << "ns)" 
                        << "\n  Frame: " <<  filtered_msg.header.frame_id
                        << "\n  Timestamp: " <<  std::to_string(TimeToDouble(filtered_msg.header.stamp))
                        << "\n  Number of points: " << filtered_points->points.size()
                        << std::endl;

                debug_ss << "    |\n    |\n    V"
                        << "\nLaserScan published" << " (node time: " << (this->now()).nanoseconds() << "ns)" 
                        << "\n  Frame: " <<  laser_scan_msg.header.frame_id
                        << "\n  Timestamp: " <<  std::to_string(TimeToDouble(laser_scan_msg.header.stamp))
                        << "\n  Number of rays: " << total_vals
                        << "\n  Number of hit: " << non_zero_vals
                        << std::endl;

                tf = (this->now()).nanoseconds();
                double delay_ms = (tf-t0)/1000000; //delay ms
                debug_ss << "Process time: " << delay_ms << " ms" 
                         << "\nCurrent rate: " << (1000.0/delay_ms) << " Hz"
                         << std::endl;


                if(debug && show_ranges){
                    debug_ss << "Ranges: ";
                    for(int i=0; i < total_vals; i++){
                        debug_ss << laser_scan_msg.ranges[i] << " ";
                    }
                    debug_ss << std::endl;
                }

                if(debug){
                    std::string debug_msg = debug_ss.str();
                    write_debug(debug_file_path, debug_msg);
                }
            }
        }
    }

    sensor_msgs::msg::LaserScan new_clean_scan() {
        sensor_msgs::msg::LaserScan clean_scan;
        clean_scan.header.frame_id=out_frame;
        clean_scan.angle_min=-h_fov/2; 
        clean_scan.angle_max=h_fov/2; 
        clean_scan.angle_increment=h_angle_increment;
        clean_scan.range_min=range_min; 
        clean_scan.range_max=range_max;
        clean_scan.time_increment=time_increment; 
        clean_scan.scan_time=scan_time; 

        int resolution = static_cast<int>(round((clean_scan.angle_max-clean_scan.angle_min)/h_angle_increment));
        //init ranges and add inf values according to the resolution
        clean_scan.ranges = {};
        for (int i = 0; i < resolution; ++i){
            clean_scan.ranges.push_back(INFINITY);
            clean_scan.intensities.push_back(0.0);
        }

        return clean_scan;
    }

    int update_transform(geometry_msgs::msg::TransformStamped &transform, std::string goal_frame, std::string init_frame){
        //return 0 if fail to intiialise transform goal_frame => init_frame and 1 else, even when fail to update.
        std::stringstream debug_ss;
        if (transform.header.stamp == rclcpp::Time(0)) { //if not initialized we try to initialize and return an error if fail
            try {
                transform = tf_buffer_->lookupTransform(goal_frame, init_frame, tf2::TimePointZero);
                return 1;
            }
            catch (const std::exception& e) {
                debug_ss  << "ERROR: Couldn't get an initial transformation '" << goal_frame << "' --> '" << init_frame << "'" << std::endl;
                std::string debug_msg = debug_ss.str();
                RCLCPP_INFO(this->get_logger(), "%s", debug_msg.c_str()); //we print errores anyway
                if(debug){
                    debug_ss << "Error details: " << e.what() << std::endl;
                    debug_msg = debug_ss.str();
                    write_debug(debug_file_path, debug_msg);
                }
                return 0;
            }
        } else { //if already initialized, we return sucess anyway but we send warning if we couldn't update
            try {
                transform = tf_buffer_->lookupTransform(goal_frame, init_frame, tf2::TimePointZero);
            }
            catch (const std::exception& e) {
                debug_ss  << "WARN: Couldn't update the transformation '" << goal_frame << "' --> '" << init_frame << "' , using last detected with timestamp: " << std::to_string(TimeToDouble(transform.header.stamp)) << " s" << std::endl;
                std::string debug_msg = debug_ss.str();
                RCLCPP_INFO(this->get_logger(), "%s", debug_msg.c_str()); //we print the warn anyway
                if(debug){
                    debug_ss << "Error details: " << e.what() << std::endl;
                    debug_msg = debug_ss.str();
                    write_debug(debug_file_path, debug_msg);
                }
            }
            return 1;
        }
    }

    void initialize_params(){
        //classic parameters gathering
        //this->declare_parameter("use_sim_time",false); //already declared by launch
        this->declare_parameter("rate", 20.0);
        this->declare_parameter("topic_in", "cam1/depth/points");
        this->declare_parameter("ref_frame", "footprint");
        this->declare_parameter("height_offset", 0.0);
        this->declare_parameter("min_height", 0.02);
        this->declare_parameter("max_height", 1.5);
        this->declare_parameter("h_fov", 1.570796327);
        this->declare_parameter("range_min", 0.0);
        this->declare_parameter("range_max", 5.0);
        this->declare_parameter("publish_cloud", true);
        this->declare_parameter("topic_out_cloud", "cam1/filtered_cloud");
        this->declare_parameter("publish_scan", true);
        this->declare_parameter("topic_out_scan", "cam1/scan");
        this->declare_parameter("out_frame", "cam1_depth_frame");
        this->declare_parameter("h_angle_increment", 0.017453293);
        this->declare_parameter("cliff_detect", true);
        this->declare_parameter("cliff_height", -0.02);
        this->declare_parameter("time_increment", 0.0);
        this->declare_parameter("scan_time", 0.0);
        this->declare_parameter("debug", true);
        this->declare_parameter("debug_file_path", "/home/jrluser/Desktop/ros_workspace/depth_to_scan_debug.txt");
        this->declare_parameter("show_ranges", true);
    }

    void refresh_params(){
        this->get_parameter("rate", rate);
        this->get_parameter("topic_in", topic_in);
        this->get_parameter("ref_frame", ref_frame);
        this->get_parameter("height_offset", height_offset);
        this->get_parameter("min_height", min_height);
        this->get_parameter("max_height", max_height);
        this->get_parameter("h_fov", h_fov);
        this->get_parameter("range_min", range_min);
        this->get_parameter("range_max", range_max);
        this->get_parameter("publish_cloud", publish_cloud);
        this->get_parameter("topic_out_cloud", topic_out_cloud);
        this->get_parameter("publish_scan", publish_scan);
        this->get_parameter("topic_out_scan", topic_out_scan);
        this->get_parameter("out_frame", out_frame);
        this->get_parameter("h_angle_increment", h_angle_increment);
        this->get_parameter("cliff_detect", cliff_detect);
        this->get_parameter("cliff_height", cliff_height);
        this->get_parameter("time_increment", time_increment);
        this->get_parameter("scan_time", scan_time);
        this->get_parameter("debug", debug);
        this->get_parameter("debug_file_path", debug_file_path);
        this->get_parameter("show_ranges", show_ranges);
    }

    void debug_params(){
        std::stringstream debug_ss;
        debug_ss <<"depth_to_scan_node started !";
        debug_ss << "\nPARAMETERS:"
                << "\nrate: " << rate
                << "\ntopic_in: " << topic_in
                << "\nref_frame: " << ref_frame
                << "\nheight_offset: " << height_offset
                << "\nmin_height: " << min_height
                << "\nmax_height: " << max_height
                << "\nh_fov: " << h_fov
                << "\nrange_min: " << range_min
                << "\nrange_max: " << range_max
                << "\npublish_cloud: " << publish_cloud
                << "\ntopic_out_cloud: " << topic_out_cloud
                << "\npublish_scan: " << publish_scan
                << "\ntopic_out_scan: " << topic_out_scan
                << "\nout_frame: " << out_frame
                << "\nh_angle_increment: " << h_angle_increment
                << "\ncliff_detect: " << cliff_detect
                << "\ncliff_height: " << cliff_height
                << "\ntime_increment: " << time_increment
                << "\nscan_time: " << scan_time
                << "\ndebug: " << debug
                << "\ndebug_file_path: " << debug_file_path
                << "\nshow_ranges: " << show_ranges;

        std::string debug_msg = debug_ss.str();
        write_debug(debug_file_path, debug_msg, false);
        RCLCPP_INFO(this->get_logger(), "%s", debug_msg.c_str());
    }

    void write_debug(std::string file, std::string text,  bool append = true){
        std::lock_guard<std::mutex> lock(mutex_debug_file);
        std::ofstream debug_file_ss(file, append ? std::ios::app : std::ios::trunc);
        if (debug_file_ss.is_open()){
            debug_file_ss << text;
            debug_file_ss.close();
            RCLCPP_INFO(this->get_logger(), "%s",text.c_str());
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Could not open file for debugging: %s",file.c_str());
        }
    }

    void ClockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
        // Update the current timestamp when the clock message is received
        simu_timestamp = msg->clock;
    }
    
    //data
    sensor_msgs::msg::PointCloud2::SharedPtr raw_msg = nullptr;
    geometry_msgs::msg::TransformStamped transf_world_cam;
    geometry_msgs::msg::TransformStamped transf_laser_cam;
    //subscribers/publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_points_publisher_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    //node settngs
    bool use_sim_time;
    double rate;
    //points cloud filtering setting
    std::string topic_in;
    std::string ref_frame;
    double height_offset;
    double min_height;
    double max_height;
    double h_fov;
    double range_min;
    double range_max;
    bool publish_cloud;
    std::string topic_out_cloud;
    //Scan settings
    bool publish_scan;
    std::string topic_out_scan;
    std::string out_frame;
    double h_angle_increment;
    bool cliff_detect;
    double cliff_height;
    //advanced output scan settings
    double time_increment;
    double scan_time;
    //debugging
    bool debug;
    std::string debug_file_path;
    bool show_ranges;
    //concurrence
    std::mutex mutex_debug_file;
    std::mutex mutex_points_cloud;
    //clock
    builtin_interfaces::msg::Time simu_timestamp; //used for simuation
    rclcpp::Clock::SharedPtr clock; //used if not a simulation
    //transformations listening
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudToLaserScanNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

Eigen::MatrixXd get_4Dmatrix_from_transform(geometry_msgs::msg::TransformStamped transf){
    geometry_msgs::msg::Vector3 translation_ = transf.transform.translation;  //translation vector from new_frame to frame_sensor
    geometry_msgs::msg::Vector3 rotation_ = quaternion_to_euler3D(transf.transform.rotation);
    //Translation in homogeneous coordinates
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
    T(0,3) = translation_.x;
    T(1,3) = translation_.y;
    T(2,3) = translation_.z;
    //3D rotation matrix
    Eigen::MatrixXd R_temp = rot_matrix_from_euler(rotation_);
    //3D rotation in homogeneous coordinates
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4);
    for(int i = 0; i<3; i++){
        for(int y = 0; y<3; y++){
            R(i,y) = R_temp(i,y);
        }
    }
    //final homogeneous transformation, M1_2 = pass matrix, displacement from new_frame to init_frame
    Eigen::MatrixXd M1_2(4, 4);
    M1_2 = T*R;
    return M1_2;
}

Eigen::MatrixXd rot_matrix_from_euler(geometry_msgs::msg::Vector3 euler_angles){
    Eigen::MatrixXd Rx = Eigen::MatrixXd::Identity(3, 3);
    Rx(1,1) = cos(euler_angles.x);
    Rx(1,2) = -sin(euler_angles.x);
    Rx(2,1) = sin(euler_angles.x);
    Rx(2,2) = cos(euler_angles.x);
    Eigen::MatrixXd Ry = Eigen::MatrixXd::Identity(3, 3);
    Ry(0,0) = cos(euler_angles.y);
    Ry(0,2) = sin(euler_angles.y);
    Ry(2,0) = -sin(euler_angles.y);
    Ry(2,2) = cos(euler_angles.y);
    Eigen::MatrixXd Rz = Eigen::MatrixXd::Identity(3, 3);
    Rz(0,0) = cos(euler_angles.z);
    Rz(0,1) = -sin(euler_angles.z);
    Rz(1,0) = sin(euler_angles.z);
    Rz(1,1) = cos(euler_angles.z);
    return Rx*Ry*Rz;
}

geometry_msgs::msg::Vector3 quaternion_to_euler3D(geometry_msgs::msg::Quaternion quat){
    // Convert geometry_msgs::msg::Quaternion to Eigen::Quaterniond
    Eigen::Quaterniond eigen_quaternion(
        quat.w,
        quat.x,
        quat.y,
        quat.z
    );
    // Convert quaternion to Euler angles
    Eigen::Vector3d euler_angles = eigen_quaternion.toRotationMatrix().eulerAngles(0, 1, 2); 
    geometry_msgs::msg::Vector3 rot_vect;
    rot_vect.x = euler_angles(0);
    rot_vect.y = euler_angles(1);
    rot_vect.z = euler_angles(2);
    //RCLCPP_INFO(this->get_logger(), "ROT Quat: x='%.2f', y='%.2f', z='%.2f', w='%.2f'", quat.x,quat.y,quat.z,quat.w);
    //RCLCPP_INFO(this->get_logger(), "ROT Euler: x='%.2f', y='%.2f', z='%.2f'", rot_vect.x,rot_vect.y,rot_vect.z);
    return rot_vect;
}

double TimeToDouble(builtin_interfaces::msg::Time& stamp){
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

bool consider_val(int current_ind, int start_ind, int end_ind){
    // return true if current_ind is between start_ind and end_ind according to a circle reference.
    if(start_ind>end_ind){ //if interval pass throught the origin of the circle, we test considering the split into 2 interval
        return (current_ind>=start_ind || current_ind<=end_ind);
    }
    else{ // if values are equal, or classical ,we test as classic interval
        return (current_ind>=start_ind && current_ind<=end_ind);
    }
}

int angle_to_index(double alpha, int resolution){
    //return index of angle alpha, in a table with 'resolution' values placed from 0 to 360 degree.
    // Normalize the angle to the range [0, 2*M_PI)
    alpha = std::fmod(alpha, 2 * M_PI);
    if (alpha < 0) {
        alpha += 2 * M_PI;
    }
    // Calculate the index
    return static_cast<int>(round((alpha * resolution) / 2*M_PI));
}

int remap_scan_index(int prev_ind, double prev_angle_start, double prev_angle_end, double prev_reso, double new_angle_start, double new_angle_end, double new_reso){
    int new_ind;
    /*
    return the index in a new scan list.
    */

   //offset gestion
    double angle_offset = sawtooth(new_angle_start-prev_angle_start,2*M_PI);
    int ind_offset = -std::copysign(1.0,angle_offset)*angle_to_index(std::abs(angle_offset),prev_reso);
    new_ind = static_cast<int>(round(fmod(prev_ind + ind_offset,prev_reso)));
    if(new_ind<0){
        new_ind += prev_reso;
    }
    //different reso gestion 
    double prev_elong = prev_angle_end - prev_angle_start;  
    double new_elong = new_angle_end - new_angle_start;
    double prev_angle_incr = prev_elong/prev_reso;
    double new_angle_incr = new_elong/new_reso;
    new_ind = static_cast<int>(round((prev_angle_incr*new_ind)/new_angle_incr));
    
    return new_ind;
}

double sawtooth(double x, double period) {
    return 2.0 * (x / period - floor(x / period)) - 1.0;
}

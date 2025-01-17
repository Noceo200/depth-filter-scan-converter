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
#include "tools.h"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono> //Only for testing to reduce speed of computation

class PointCloudToLaserScanNode : public rclcpp::Node
{
public:
    PointCloudToLaserScanNode() : Node("depth_filter_scan_converter_node")
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
            filtered_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_out_cloud, default_qos);
        }

        // Initialize subscriber to odometry
        if(compensate_move){
            odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, sensor_qos, std::bind(&PointCloudToLaserScanNode::odomCallback, this, std::placeholders::_1));
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

        //Update message
        mutex_points_cloud.lock();
        raw_msg = msg;
        mutex_points_cloud.unlock();

        debug_ss << "\n\n" << "Data received... (timestamp: " << std::to_string(TimeToDouble(raw_msg->header.stamp)) <<" s)"<<" (node time: " << (this->now()).nanoseconds() << "ns)" << std::endl;

        //debug
        if(debug){
            std::string debug_msg = debug_ss.str();
            write_debug(debug_file_path, debug_msg);
        }

        //Update transformations
        mutex_transforms.lock();
        cloud_ready = 0;
        cloud_ready2 = 0;
        cloud_ready = update_transform(transf_cam_ref,raw_msg->header.frame_id,ref_frame,raw_msg->header.stamp); //needed even if we don't publish the clouds
        cloud_ready2 = update_transform(transf_ref_cam,ref_frame,raw_msg->header.frame_id,raw_msg->header.stamp); //same but inverted transformation to avoid to ave to invert matrixes
        mutex_transforms.unlock();

        //update odometry
        if(compensate_move){
            mutex_odom.lock();
            odom_msg_former = odom_msg_new;
            mutex_odom.unlock();
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        //Update odometry
        mutex_odom.lock();
        odom_msg_new = msg;
        mutex_odom.unlock();
    }

    void DepthFilterToScan(){
        std::stringstream debug_ss;

        if(raw_msg != nullptr && TimeToDouble(raw_msg->header.stamp) != last_timestamp){
            int t0;
            int tf;
            t0 = (this->now()).nanoseconds();
            
            // Copy data
            mutex_points_cloud.lock();
            sensor_msgs::msg::PointCloud2::SharedPtr msg(new sensor_msgs::msg::PointCloud2(*raw_msg));
            mutex_points_cloud.unlock();
            last_timestamp = TimeToDouble(msg->header.stamp);

            //convert PointCloud2 to points pcl object
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*msg, pcl_pc2);
            pcl::PointCloud<pcl::PointXYZ>::Ptr raw_points(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(pcl_pc2, *raw_points);

            //common initialisation
            int nb_points = raw_points->points.size();
            int nb_column = msg->width;
            int nb_line = msg->height;
            if(nb_line==1){ //if we have uordered list of points, we consider it as a square cloud
                nb_column = static_cast<int>(sqrt(nb_column));
                nb_line = nb_column;
            }

            mutex_transforms.lock();
            bool can_compute = cloud_ready && cloud_ready2;
            mutex_transforms.unlock();

            if(can_compute){

                //version with multidimensionnal array in trash below, but slower for initialisaton

                //filter initialisation
                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points(new pcl::PointCloud<pcl::PointXYZ>);
                std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> Pass_matrixes_cam_ref = get_4Dmatrix_from_transform(transf_cam_ref); //needed anyway by both feature, here we just get the rotation matrix
                std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> Pass_matrixes_ref_cam = get_4Dmatrix_from_transform(transf_ref_cam); //needed anyway by both feature, here we just get the rotation matrix

                //scan initialisation
                int scan_reso = static_cast<int>(round((angle_max-angle_min)/h_angle_increment));
                int circle_reso = static_cast<int>(round(scan_reso*(2*M_PI/(angle_max-angle_min))));
                int start_index = angle_to_index(angle_min, circle_reso); //start index acording to fov on a 360 degree scan with circle_reso values, we take 360 so that we can throw points that are somewhere else on the circle
                int end_index = angle_to_index(angle_max, circle_reso);
                sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg = new_clean_scan(); // To convert PointCloud2 to LaserScan

                //debug initialisation
                int nb_inbound_points = 0;
                int nb_cliff_points = 0;
                int added_as_obstacles = 0;
                int nb_thrown_points = 0;

                double height_cam_offset = std::get<0>(Pass_matrixes_ref_cam)(2,3); 
                double x_cam_offset = std::get<0>(Pass_matrixes_ref_cam)(0,3); 
                double y_cam_offset = std::get<0>(Pass_matrixes_ref_cam)(1,3);

                //constraints initialisation
                //wanted constraints vector in ref frame
                Eigen::MatrixXd C_z = Eigen::MatrixXd::Identity(4, 1); //direction of height constraint on vector z
                C_z(0,0) = 0.0;
                C_z(1,0) = 0.0;
                C_z(2,0) = 1.0;
                C_z(3,0) = 1.0;
                Eigen::MatrixXd C_x = Eigen::MatrixXd::Identity(4, 1); //direction of range constraint on vector x
                C_x(0,0) = 1.0;
                C_x(1,0) = 0.0;
                C_x(2,0) = 0.0;
                C_x(3,0) = 1.0;
                Eigen::MatrixXd C_y = Eigen::MatrixXd::Identity(4, 1); //need to define y vector to know angle positivity
                C_y(0,0) = 0.0;
                C_y(1,0) = 1.0;
                C_y(2,0) = 0.0;
                C_y(3,0) = 1.0;
                //equivalent constraint vector in camera frame, we apply just the rotation, we just need directions
                Eigen::MatrixXd C_z_f = std::get<1>(Pass_matrixes_cam_ref)*C_z;
                Eigen::MatrixXd C_x_f = std::get<1>(Pass_matrixes_cam_ref)*C_x;
                Eigen::MatrixXd C_y_f = std::get<1>(Pass_matrixes_cam_ref)*C_y;
                //3d versions, using multi dimensionnal arrays are faster inside the coming loop
                double C_z_bar[3] = {C_z_f(0,0),C_z_f(1,0),C_z_f(2,0)};
                double C_x_bar[3] = {C_x_f(0,0),C_x_f(1,0),C_x_f(2,0)};
                double C_y_bar[3] = {C_y_f(0,0),C_y_f(1,0),C_y_f(2,0)};

                debug_ss << "\nStarting process for "<< int(nb_points/(speed_up_h*speed_up_v)) << " points (timestamp: "<< std::to_string(TimeToDouble(msg->header.stamp)) <<" s)" << " (node time: " << (this->now()).nanoseconds() << "ns)" << std::endl;

                // Iterate over each point in the PointCloud and fill filtered_points and laserscan
                for (int il =0; il<nb_line; il+=speed_up_h)
                {
                    for (int ic =0; ic<nb_column; ic+=speed_up_v){
                        int i = il*nb_column+ic; //position of point in the list

                        auto point = raw_points->points[i];

                        /*if(i%50000 == 0){
                            debug_ss << "\nDEBUG_PERSO: point coordinates: (" << point.x << "," << point.y << "," << point.z << ")" << std::endl;
                        }*/

                        // Access the coordinates of the point in its frame, homogenous coordinates
                        double P_parent[3] = {point.x,point.y,point.z}; //faster than using Eigen in the loop

                        // Process the point here...
                        double z_cam = scalar_projection_fast(P_parent,C_z_bar); //z coordinate in world orienttion but centered on cam_frame
                        double z_world = height_offset + z_cam + height_cam_offset; //Height in world = height offset between ref_frame and floor + height of points in camera_frame + height of camera in ref_frame (all following the axis z of ref_frame)
                        double x_cam = scalar_projection_fast(P_parent,C_x_bar);  //x coordinate in world orienttion but centered on cam_frame
                        double y_cam = scalar_projection_fast(P_parent,C_y_bar); //y coordinate in world orienttion but centered on cam_frame
                        double x_ref; //x coordinate in ref_frame, can be changed in case of cliff
                        double y_ref; //y coordinate in ref_frame, can be changed in case of cliff

                        bool is_cliff = false;
                        if(cliff_detect && z_world <= cliff_height){//also work for points at infinite distance
                            is_cliff = true;
                            //The obstacle is situated closer than the detected point (especially for point at infinite distance)
                            //So we recompute the distance
                            double new_z = height_offset + height_cam_offset; //height we want to apply for Thales and have the obstacle at this distance
                            double dist_cam = sqrt(pow(x_cam,2)+pow(y_cam,2));
                            double new_dist_cam = new_z*dist_cam/abs(z_cam);
                            double K = new_dist_cam/dist_cam;
                            x_ref = x_cam_offset+K*x_cam;
                            y_ref = y_cam_offset+K*y_cam;
                        }
                        else{
                            x_ref = x_cam + x_cam_offset;
                            y_ref = y_cam + y_cam_offset;
                        }

                        double angle_ref = atan2(y_ref,x_ref); //angle in ref frame
                        int ind_circle_ref = angle_to_index(angle_ref,circle_reso); //index where it should be in a 360 degree laserscan list of circle_reso values centered on ref_frame
                        float dist = sqrt(pow(x_ref,2)+pow(y_ref,2)); //planar distance from robot center
                        bool in_bounds = (min_height <= z_world) && (max_height >= z_world) && consider_val(ind_circle_ref, start_index, end_index) && (range_min <= dist) && (range_max >= dist);

                        if(in_bounds || is_cliff){
                            //fill cloud
                            if(publish_cloud){
                                filtered_points->push_back(point);
                            } 
                            //fill scan
                            if(publish_scan){
                                int real_ind = remap_scan_index(ind_circle_ref, 0.0, 2*M_PI, circle_reso, angle_min, angle_max, scan_reso); //we want the index for a list that represent values from angle_min values to angle_max values. because the laserscan message is configured like this
                                if(consider_val(real_ind,0,scan_reso-1)){
                                    if(dist<laser_scan_msg->ranges[real_ind]){
                                        laser_scan_msg->ranges[real_ind] = dist;
                                        if(!is_cliff){
                                            laser_scan_msg->intensities[real_ind] = std::max(0.0,(z_world-min_height)/(max_height-min_height)); //proportionnal to height interval selected by user
                                        }
                                        else{
                                            laser_scan_msg->intensities[real_ind] = -1.0;
                                        }
                                    }
                                    added_as_obstacles += 1;
                                }
                                //other cases should never happen as we selected the points that should be in this laser scan already with 'in_bounds', but we add the condition just in case extremities indexes cause problems.
                            }
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

                }

                //std::this_thread::sleep_for(std::chrono::seconds(2)); 

                //move compensation
                if(compensate_move){
                    //TDM need to be updated with odometry list method used in scan merger or deleted
                    mutex_odom.lock();
                    nav_msgs::msg::Odometry::SharedPtr current_odom(new nav_msgs::msg::Odometry(*odom_msg_new)); //current odom
                    mutex_odom.unlock();
                    //we get new position and Heading that might have changed due to computation time
                    if(odom_msg_former!=nullptr && current_odom!=nullptr){
                        geometry_msgs::msg::Vector3 euler_heading_former = adapt_angle(quaternion_to_euler3D(odom_msg_former->pose.pose.orientation)); //previous heading
                        geometry_msgs::msg::Vector3 euler_heading_new = adapt_angle(quaternion_to_euler3D(odom_msg_new->pose.pose.orientation)); //new heading
                        double off_x = current_odom->pose.pose.position.x - odom_msg_former->pose.pose.position.x;
                        double off_y = current_odom->pose.pose.position.y - odom_msg_former->pose.pose.position.y;
                        double off_tetha = sawtooth(euler_heading_new.z - euler_heading_former.z);
                        debug_ss << "Former Odometry: (x,y,tetha): (" << odom_msg_former->pose.pose.position.x << "," << odom_msg_former->pose.pose.position.y << "," << rads_to_degrees(euler_heading_former.z) << ") m, deg (timestamp: " << std::to_string(TimeToDouble(odom_msg_former->header.stamp)) << " s)" << std::endl;
                        debug_ss << "New Odometry: (x,y,tetha): (" << current_odom->pose.pose.position.x << "," << current_odom->pose.pose.position.y << "," << rads_to_degrees(euler_heading_new.z) << ") m, deg (timestamp: " << std::to_string(TimeToDouble(current_odom->header.stamp)) << " s)" << std::endl;
                        if(off_x != 0.0 || off_y != 0.0 || off_tetha != 0.0){
                            transform_opened_scan(laser_scan_msg, -off_x, -off_y, -off_tetha, debug_ss);
                            debug_ss << "Scan adjusted. (x,y,tetha) offset: (" << off_x << "," << off_y << "," << rads_to_degrees(off_tetha) << ") m, deg " << std::endl;
                        }
                        else{
                            debug_ss << "No need to compensate and adjust the scan, position and heading didn't change during computation time." << std::endl;
                        }
                    }
                    else{
                        debug_ss << "WARN: Couldn't compensate motion and adjust the scan: No odometry messages received from topic '" << odom_topic << "'" << std::endl;
                    }
                }

                // Convert the filtered point cloud to a ROS message
                sensor_msgs::msg::PointCloud2 filtered_msg;
                pcl::toROSMsg(*filtered_points, filtered_msg);
                filtered_msg.header = msg->header; // Set the header

                //put clock
                update_stamp();
                laser_scan_msg->header.stamp = DoubleToTime(last_timestamp); //current_global_stamp;
                filtered_msg.header.stamp = DoubleToTime(last_timestamp);

                //Publish filtered cloud points
                if(publish_cloud){
                    filtered_points_publisher_->publish(filtered_msg);
                }

                int non_zero_vals = 0;
                int total_vals = 0;
                if(publish_scan){
                    // Publish the LaserScan message
                    laser_scan_publisher_->publish(*laser_scan_msg);
                    //debug data
                    if(debug){
                        total_vals = laser_scan_msg->ranges.size();
                        for(int i=0; i < total_vals; i++){
                            if(laser_scan_msg->ranges[i] < INFINITY){
                                non_zero_vals ++;
                            }
                        }
                    }
                }

                //debug
                debug_ss << "    |\n    |\n    V"
                        << "\nWe keep points in these bounds: "
                        << "\n  - Angles: [" << angle_min << "," << angle_max << "] rad"
                        << "\n  - Height: [" << min_height << "," << max_height << "] m"
                        << "\n  - Or hole with height (if cliff_detect = true): < " << cliff_height << " m"
                        << "\n  - Radius: [" << range_min << "," << range_max << "] m"
                        << "\nPoints results: "
                        << "\n   Points in bounds: " << nb_inbound_points
                        << "\n   Holes: " << nb_cliff_points
                        << "\n   Total Obstacles: " << added_as_obstacles
                        << "\n   Ignored points: " << nb_thrown_points
                        << std::endl;

                if(publish_cloud){
                    debug_ss << "    |\n    |\n    V"
                            << "\nFiltered points cloud published" << " (node time: " << (this->now()).nanoseconds() << "ns)" 
                            << "\n  Frame: " <<  filtered_msg.header.frame_id
                            << "\n  Timestamp: " <<  std::to_string(TimeToDouble(filtered_msg.header.stamp))
                            << "\n  Number of points: " << filtered_points->points.size()
                            << std::endl;
                }

                if(publish_scan){
                    debug_ss << "    |\n    |\n    V"
                            << "\nLaserScan published" << " (node time: " << (this->now()).nanoseconds() << "ns)" 
                            << "\n  Frame: " <<  laser_scan_msg->header.frame_id
                            << "\n  Timestamp: " <<  std::to_string(TimeToDouble(laser_scan_msg->header.stamp))
                            << "\n  Number of rays: " << total_vals
                            << "\n  Number of hit: " << non_zero_vals
                            << std::endl;
                }

                tf = (this->now()).nanoseconds();
                double delay_ms = (tf-t0)/1000000; //delay ms
                debug_ss << "    |\n    V"
                         << "\nProcess time: " << delay_ms << " ms" 
                         << "\nMaximum possible rate: " << (1000.0/delay_ms) << " Hz"
                         << std::endl;


                if(debug && show_ranges && publish_scan){
                    debug_ss << "Ranges: ";
                    for(int i=0; i < total_vals; i++){
                        debug_ss << laser_scan_msg->ranges[i] << " ";
                    }
                    debug_ss << std::endl;
                }

            }
        }
        else{
            debug_ss << "\nWaiting to receive points cloud on topic '" << topic_in << "'..." << std::endl;
        }

        if(debug){
            std::string debug_msg = debug_ss.str();
            write_debug(debug_file_path, debug_msg);
        }
    }

    sensor_msgs::msg::LaserScan::SharedPtr new_clean_scan() {
        sensor_msgs::msg::LaserScan::SharedPtr clean_scan =  std::make_shared<sensor_msgs::msg::LaserScan>();
        clean_scan->header.frame_id=ref_frame;
        clean_scan->angle_min=angle_min;
        clean_scan->angle_max=angle_max; 
        clean_scan->angle_increment=h_angle_increment;
        clean_scan->range_min=range_min; 
        clean_scan->range_max=range_max;
        clean_scan->time_increment=time_increment; 
        clean_scan->scan_time=scan_time; 

        int resolution = static_cast<int>(round((clean_scan->angle_max-clean_scan->angle_min)/h_angle_increment));
        //init ranges and add inf values according to the resolution
        clean_scan->ranges = {};
        for (int i = 0; i < resolution; ++i){
            clean_scan->ranges.push_back(INFINITY);
            clean_scan->intensities.push_back(0.0);
        }

        return clean_scan;
    }

    int update_transform(geometry_msgs::msg::TransformStamped &transform, std::string goal_frame, std::string init_frame, builtin_interfaces::msg::Time ref_time){
        //return 0 if fail to intiialise transform goal_frame => init_frame and 1 else, even when fail to update. Also if the transformation is older than min_timestamp we return 2 and write a warning.
        std::stringstream debug_ss;
        std::string debug_msg;
        int result = 0;
        rclcpp::Time ref_rcl_time(ref_time.sec, ref_time.nanosec, RCL_ROS_TIME);
        if (transform.header.stamp == rclcpp::Time(0)) { //if not initialized we try to initialize and return an error if fail
            try {
                transform = tf_buffer_->lookupTransform(goal_frame, init_frame, ref_rcl_time);
                debug_ss  << "Transformation '" << goal_frame << "' --> '" << init_frame << "' Initialized for first time. (timestamp: "<< std::to_string(TimeToDouble(transform.header.stamp)) << " s)" << std::endl;
                result = 1;
            }
            catch (const std::exception& e) {
                debug_ss  << "ERROR: Couldn't get an initial transformation '" << goal_frame << "' --> '" << init_frame << "'" << std::endl;
                debug_msg = debug_ss.str();
                RCLCPP_INFO(this->get_logger(), "%s", debug_msg.c_str()); //we print errores anyway
                debug_ss << "Error details: " << e.what() << std::endl;
                result = 0;
            }
        } else { //if already initialized, we return sucess anyway but we send warning if we couldn't update
            try {
                transform = tf_buffer_->lookupTransform(goal_frame, init_frame, ref_rcl_time);
                debug_ss  << "Transformation '" << goal_frame << "' --> '" << init_frame << "' Updated. (timestamp: "<< std::to_string(TimeToDouble(transform.header.stamp)) << " s)"<< std::endl;
                result = 1;
            }
            catch (const std::exception& e) {
                debug_ss  << "WARN: Couldn't update the transformation '" << goal_frame << "' --> '" << init_frame << "', trying to get last published one..." << std::endl; 
                debug_msg = debug_ss.str();
                //RCLCPP_WARN(this->get_logger(), "%s", debug_msg.c_str()); //we print the warn anyway
                debug_ss << "Error details: " << e.what() << std::endl;
                result =2;
            }
        }

        if(result==2 || result==0){
            try {
                transform = tf_buffer_->lookupTransform(goal_frame, init_frame, tf2::TimePointZero);
                debug_ss  << "Last Published transformation '" << goal_frame << "' --> '" << init_frame << "' Got. (timestamp: "<< std::to_string(TimeToDouble(transform.header.stamp)) << " s)"<< std::endl;
            }
            catch (const std::exception& e) {
                debug_ss  << "WARN: Couldn't get last published transformation '" << goal_frame << "' --> '" << init_frame << "', using last updated one with timestamp: " << std::to_string(TimeToDouble(transform.header.stamp)) << " s" << std::endl;
                debug_msg = debug_ss.str();
                //RCLCPP_WARN(this->get_logger(), "%s", debug_msg.c_str()); //we print the warn anyway
                debug_ss << "Error details: " << e.what() << std::endl;
            }
        }

        if(debug){
            debug_msg = debug_ss.str();
            write_debug(debug_file_path, debug_msg);
        }

        return result;

    }

    void initialize_params(){
        //classic parameters gathering
        //this->declare_parameter("use_sim_time",false); //already declared by launch
        this->declare_parameter("rate", 20.0);
        this->declare_parameter("topic_in", "cam1/depth/points");
        this->declare_parameter("ref_frame", "base_link");
        this->declare_parameter("height_offset", 0.0);
        this->declare_parameter("min_height", 0.02);
        this->declare_parameter("max_height", 1.5);
        this->declare_parameter("angle_min", -0.785398163);
        this->declare_parameter("angle_max", 0.785398163);
        this->declare_parameter("range_min", 0.0);
        this->declare_parameter("range_max", 5.0);
        this->declare_parameter("speed_up_h", 1);
        this->declare_parameter("speed_up_v", 1);
        this->declare_parameter("compensate_move", true);
        this->declare_parameter("odom_topic", "odom");
        this->declare_parameter("publish_cloud", true);
        this->declare_parameter("topic_out_cloud", "cam1/filtered_cloud");
        this->declare_parameter("publish_scan", true);
        this->declare_parameter("topic_out_scan", "cam1/scan");
        this->declare_parameter("h_angle_increment", 0.017453293);
        this->declare_parameter("cliff_detect", true);
        this->declare_parameter("cliff_height", -0.02);
        this->declare_parameter("time_increment", 0.0);
        this->declare_parameter("scan_time", 0.0);
        this->declare_parameter("debug", true);
        this->declare_parameter("debug_file_path", "depth_filter_scan_converter_debug.txt");
        this->declare_parameter("show_ranges", true);
    }

    void refresh_params(){
        this->get_parameter("rate", rate);
        this->get_parameter("topic_in", topic_in);
        this->get_parameter("ref_frame", ref_frame);
        this->get_parameter("height_offset", height_offset);
        this->get_parameter("min_height", min_height);
        this->get_parameter("max_height", max_height);
        this->get_parameter("angle_min", angle_min);
        this->get_parameter("angle_max", angle_max);
        this->get_parameter("range_min", range_min);
        this->get_parameter("range_max", range_max);
        this->get_parameter("speed_up_h", speed_up_h);
        this->get_parameter("speed_up_v", speed_up_v);
        this->get_parameter("compensate_move", compensate_move);
        this->get_parameter("odom_topic", odom_topic);
        this->get_parameter("publish_cloud", publish_cloud);
        this->get_parameter("topic_out_cloud", topic_out_cloud);
        this->get_parameter("publish_scan", publish_scan);
        this->get_parameter("topic_out_scan", topic_out_scan);
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
        debug_ss <<"depth_filter_scan_converter_node started !";
        debug_ss << "\nPARAMETERS:"
                << "\nrate: " << rate
                << "\ntopic_in: " << topic_in
                << "\nref_frame: " << ref_frame
                << "\nheight_offset: " << height_offset
                << "\nmin_height: " << min_height
                << "\nmax_height: " << max_height
                << "\nangle_min: " << angle_min
                << "\nangle_max: " << angle_max
                << "\nrange_min: " << range_min
                << "\nrange_max: " << range_max
                << "\nspeed_up_h: " << speed_up_h
                << "\nspeed_up_v: " << speed_up_v
                << "\ncompensate_move: " << compensate_move
                << "\nodom_topic: " << odom_topic
                << "\npublish_cloud: " << publish_cloud
                << "\ntopic_out_cloud: " << topic_out_cloud
                << "\npublish_scan: " << publish_scan
                << "\ntopic_out_scan: " << topic_out_scan
                << "\nh_angle_increment: " << h_angle_increment
                << "\ncliff_detect: " << cliff_detect
                << "\ncliff_height: " << cliff_height
                << "\ntime_increment: " << time_increment
                << "\nscan_time: " << scan_time
                << "\ndebug: " << debug
                << "\ndebug_file_path: " << debug_file_path
                << "\nshow_ranges: " << show_ranges
                << std::endl;

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
            //RCLCPP_INFO(this->get_logger(), "%s",text.c_str());
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Could not open file for debugging: %s",file.c_str());
        }
    }

    void update_stamp(){
        if(use_sim_time){ //if use_sim_time = true
            current_global_stamp = simu_timestamp;
        }
        else{
            current_global_stamp = clock->now();
        }
    }

    void ClockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
        // Update the current timestamp when the clock message is received
        simu_timestamp = msg->clock;
    }
    
    //data
    sensor_msgs::msg::PointCloud2::SharedPtr raw_msg = nullptr;
    geometry_msgs::msg::TransformStamped transf_cam_ref;
    geometry_msgs::msg::TransformStamped transf_ref_cam;
    int cloud_ready; //check if we could initialize or update transf_cam_ref
    int cloud_ready2; //check if we could initialize or update transf_ref_cam
    double last_timestamp; //last date of received raw_msg
    nav_msgs::msg::Odometry::SharedPtr odom_msg_former;
    nav_msgs::msg::Odometry::SharedPtr odom_msg_new;
    //subscribers/publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_; 
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
    double angle_min;
    double angle_max;
    double range_min;
    double range_max;
    int speed_up_h;
    int speed_up_v;
    bool compensate_move;
    std::string odom_topic;
    bool publish_cloud;
    std::string topic_out_cloud;
    //Scan settings
    bool publish_scan;
    std::string topic_out_scan;
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
    std::mutex mutex_transforms;
    std::mutex mutex_odom;
    //clock
    builtin_interfaces::msg::Time simu_timestamp; //used for simuation
    rclcpp::Clock::SharedPtr clock; //used if not a simulation
    builtin_interfaces::msg::Time current_global_stamp;
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

//trash

                /*//filter initialisation
                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points(new pcl::PointCloud<pcl::PointXYZ>);
                double Pass_matrix_cam_ref[4][4];
                get_4Dmatrix_from_transform_fast(Pass_matrix_cam_ref,transf_cam_ref); //needed anyway by both feature

                //scan initialisation
                int scan_reso = static_cast<int>(round(h_fov/h_angle_increment));
                int circle_reso = static_cast<int>(round(scan_reso*(2*M_PI/h_fov)));
                int start_index = angle_to_index(-h_fov/2, circle_reso); //start index acording to fov on a 360 degree scan with circle_reso values, we take 360 so that we can throw points that are somewhere else on the circle
                int end_index = angle_to_index(h_fov/2, circle_reso);
                sensor_msgs::msg::LaserScan laser_scan_msg = new_clean_scan(); // To convert PointCloud2 to LaserScan
                double Pass_matrix_laser_cam[4][4];
                if(scan_ready){get_4Dmatrix_from_transform_fast(Pass_matrix_laser_cam,transf_laser_cam);}

                //debug initialisation
                int nb_inbound_points = 0;
                int nb_cliff_points = 0;
                int added_as_obstacles = 0;
                int nb_thrown_points = 0;
                //constraints initialisation
                //wanted constraints vector in world frame
                double C_h[4][1] = {
                    {0.0},
                    {0.0},
                    {1.0},
                    {1.0}
                    }; //height constraint on vector z
                double C_r[4][1] = {
                    {1.0},
                    {0.0},
                    {0.0},
                    {1.0}
                    };  //range constraint on vector x
                //equivalent constraint vector in camera frame
                double C_h_f[4][1];
                MatProd_fast4_Vect(C_h_f,Pass_matrix_cam_ref,C_h);
                double C_r_f[4][1];
                MatProd_fast4_Vect(C_r_f,Pass_matrix_cam_ref,C_r);
                //3d versions
                double C_h_bar[3] = {C_h_f[0][0],C_h_f[1][0],C_h_f[2][0]};
                double C_r_bar[3] = {C_r_f[0][0],C_r_f[1][0],C_r_f[2][0]};*/
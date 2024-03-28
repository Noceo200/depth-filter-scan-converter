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
double scalar_projection(const Eigen::VectorXd& a, const Eigen::VectorXd& b);
double scalar_projection_fast(double[3] &a, double[3] &b);
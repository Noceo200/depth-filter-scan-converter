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

geometry_msgs::msg::TransformStamped tf_offset(geometry_msgs::msg::TransformStamped &relative_transform, geometry_msgs::msg::TransformStamped tf1, geometry_msgs::msg::TransformStamped tf2);
void shift_cloud_from_tf(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, geometry_msgs::msg::TransformStamped transf);
void MatProd_fast4_4(double (&M)[4][4],double (&A)[4][4],double (&B)[4][4]);
void MatProd_fast3_3(double (&M)[3][3],double (&A)[3][3],double (&B)[3][3]);
void MatProd_fast4_Vect(double (&M)[4][1],double (&A)[4][4],double (&B)[4][1]);
void to_identity3_3(double (&mat)[3][3]);
void to_identity4_4(double (&mat)[4][4]);
double TimeToDouble(builtin_interfaces::msg::Time& stamp);
bool consider_val(int current_ind, int start_ind, int end_ind);
int angle_to_index(double alpha, int resolution);
int remap_scan_index(int prev_ind, double prev_angle_start, double prev_angle_end, double prev_reso, double new_angle_start, double new_angle_end, double new_reso);
double sawtooth(double x);
geometry_msgs::msg::Vector3 quaternion_to_euler3D(geometry_msgs::msg::Quaternion quat);
std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> get_4Dmatrix_from_transform(geometry_msgs::msg::TransformStamped transf);
void get_4Dmatrix_from_transform_fast(double (&M)[4][4],geometry_msgs::msg::TransformStamped transf);
Eigen::MatrixXd rot_matrix_from_euler(geometry_msgs::msg::Vector3 euler_angles);
void rot_matrix_from_euler_fast(double (&R)[3][3], geometry_msgs::msg::Vector3 euler_angles);
double scalar_projection(const Eigen::VectorXd& a, const Eigen::VectorXd& b);
double scalar_projection_fast(double (&a)[3], double (&b)[3]);
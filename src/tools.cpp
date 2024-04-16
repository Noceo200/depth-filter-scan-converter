/*
LAST MODIF(DD/MM/YYYY): 16/04/2024
*/

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

int transform_opened_scan(sensor_msgs::msg::LaserScan::SharedPtr to_transform_scan, double off_vect_x,double off_vect_y, double off_tetha, std::stringstream &debug_ss){
    /*
    This function transform an 'opened' scan wich mean that the values will not 'slid' on a circle frame, but will be cut instead.
    to_transform_scan: current message to transform
    off_vect_x: vector new_origin=>lidar
    off_vect_y: vector new_origin=>lidar
    return transformed_scan: message in which datas will be stored
    Homogoneous transformations: https://www.fil.univ-lille.fr/portail/archive21-22/~aubert/m3d/m3d_transformation.pdf
    */
    //RCLCPP_INFO(this->get_logger(), "Transform_data: params : offx='%f'; offy='%f'; off_tetha='%f'",off_vect_x,off_vect_y,off_tetha);
    //RCLCPP_INFO(this->get_logger(), "transform 1");
    sensor_msgs::msg::LaserScan::SharedPtr transformed_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    transformed_scan->ranges.clear();
    int resolution = to_transform_scan->ranges.size();

    // Transform the LaserScan message
    for (int i = 0; i < resolution; ++i) { //initiate
        transformed_scan->ranges.push_back(INFINITY);
    }

    //let's be sure to manipulate angles from 0 to 2pi
    if (off_tetha<0){
        off_tetha=2*M_PI+off_tetha;
    }
    if (off_tetha == 2*M_PI){
        off_tetha=0;
    }
    //translation in homogeneous coordinates
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
    T(0,3) = off_vect_x;
    T(1,3) = off_vect_y;
    //rotation in homogeneous coordinates
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4);
    R(0,0) = cos(off_tetha);
    R(0,1) = -sin(off_tetha);
    R(1,0) = sin(off_tetha);
    R(1,1) = cos(off_tetha);
    //final homogeneous transformation, M1_2 = pass matrix, displacement from new_frame to sensor_frame
    Eigen::MatrixXd M1_2(4, 4);
    M1_2 = T*R;

    //debug_ss << "DEBUG: Pass Matrix:" << std::endl;
    //debug_ss << M1_2 << std::endl;

    //loop variables
    double init_x = 0;
    double init_y = 0;
    double init_val = 0;
    double init_angle = 0;
    double new_x = 0;
    double new_y = 0;
    double new_val = 0;
    double new_angle = 0;
    int new_index = 0;
    double max_angle = to_transform_scan->angle_max;
    double min_angle = to_transform_scan->angle_min;
    double elong = max_angle-min_angle;
    double angle_incr = elong/resolution; //can also be obtain with laserscan message

    for (int ind = 0; ind < resolution; ++ind) { //transform
        init_angle = index_to_angle(ind,resolution,elong); //should be between 0.0 and elongation, but will not coresspond to real angle because of min_angle offset
        init_val = to_transform_scan->ranges[ind];

        //points transformation
        if (!isinf(init_val)){ //if it is infinity, we don't need to compute as the default values in the new scan will be INFINITY
            get_pos(init_x, init_y,init_angle,init_val,0.0,0.0); //position in inital frame, vector: lidar=>point
            Eigen::MatrixXd X2(4, 1); //pos in sensor frame
            X2(0,0) = init_x;
            X2(1,0) = init_y;
            X2(2,0) = 0.0;
            X2(3,0) = 1.0;
            Eigen::MatrixXd X1(4, 1); //pos in newframe
            X1 = M1_2*X2;
            //Unpack
            double new_x2 = X1(0,0);
            double new_y2 = X1(1,0);
            new_angle = atan2(new_y2,new_x2);
            //we want angles from 0 to 2pi
            if (new_angle<0){
                new_angle=2*M_PI+new_angle;
            }
            if (new_angle == 2*M_PI){
                new_angle=0;
            }
            //RCLCPP_INFO(this->get_logger(), "Transform_data: newangle='%f'",new_angle);

            //new norm
            new_val = sqrt(pow(new_x2,2)+pow(new_y2,2));
            //new angle is between 0.0 and 2Pi so it can be outside our initial list
            new_index = angle_to_index(new_angle,static_cast<int>(round(2*M_PI/angle_incr))); //we want index as it would be if it was a 360 closed scan
            //and we crop the indexes outside the initial list
            if(new_index < resolution){
                transformed_scan->ranges[new_index] = std::min(static_cast<double>(transformed_scan->ranges[new_index]),new_val); //in case some area are no more accessible from the new origin
                //debug_ss << "Transform_data: initindex='" << ind << "'; initangle='" << init_angle << "'; initval='" << init_val << "'; initx='" << init_x << "'; inity='" << init_y << "'; newx2='" << new_x2 << "'; newy2='" << new_y2 << "'; newval='" << new_val << "'; newangle='" << new_angle << "'; newindex='" << new_index << "'";
            }
        }

    }
    //RCLCPP_INFO(this->get_logger(), "transform 3");

    //copy result ranges into to_transform_scan
    if(copy_ranges(to_transform_scan,transformed_scan)){ 
        return 1;
    } 
    else{
        debug_ss << "ERROR: filter_360_data: " << "Copy ranges failed" << '\n';
        return 0;
    }
}

double rads_to_degrees(double rads){
    return rads*180/M_PI;
}

double degrees_to_rads(double degrees){
    return degrees*M_PI/180;
}

int filter_360_data(sensor_msgs::msg::LaserScan::SharedPtr to_transform_scan,double start_angle, double end_angle, double angle_origin_offset, double min_range, double max_range, std::stringstream &debug_ss){
    /*
    Receive a to_transform_scan of a 360 scan only, the shift will not work if the scan message is not a 360 one.
    */    

    int resolution = to_transform_scan->ranges.size();
    //we shift all the values to bring back the origin angle to the x axis
    int index_shift = angle_to_index(angle_origin_offset,resolution); //index which is the first angle for the lidar
    sensor_msgs::msg::LaserScan::SharedPtr shifted_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    shifted_scan->ranges.clear();
    for (int i = 0; i < resolution; ++i) { 
        if(i+index_shift < resolution){ 
            shifted_scan->ranges.push_back(to_transform_scan->ranges[i+index_shift]);
        }
        else{
            shifted_scan->ranges.push_back(to_transform_scan->ranges[i+index_shift-resolution]);
        }
    }

    //we keep the wanted angles
    sensor_msgs::msg::LaserScan::SharedPtr filtered_scan = std::make_shared<sensor_msgs::msg::LaserScan>();;
    filtered_scan->ranges.clear();

    //compute index to consider according to min/max angles wanted
    int start_index = angle_to_index(start_angle, resolution);
    int end_index = angle_to_index(end_angle, resolution);

    // LaserScan message filling
    for (int i = 0; i < resolution; ++i) { 
        float dist = shifted_scan->ranges[i];
        if(consider_val(i, start_index, end_index) && dist<=max_range && dist>=min_range){ //if the value is autorized, we add it
            filtered_scan->ranges.push_back(shifted_scan->ranges[i]);
        }
        else{
            filtered_scan->ranges.push_back(INFINITY);
        }
    }

    //copy result ranges into to_transform_scan
    if(copy_ranges(to_transform_scan,filtered_scan)){ 
        return 1;
    } 
    else{
        debug_ss << "ERROR: filter_360_data: " << "Copy ranges failed" << '\n';
        return 0;
    }
}

void get_pos(double &x, double &y,double alpha,double val,double x_off,double y_off){
    x = val*cos(alpha)+x_off;
    y = val*sin(alpha)+y_off;
}

int transform_360_data(sensor_msgs::msg::LaserScan::SharedPtr to_transform_scan, double off_vect_x,double off_vect_y, double off_tetha, std::stringstream &debug_ss){
    /*
    to_transform_scan: current message to transform
    off_vect_x: vector new_origin=>lidar
    off_vect_y: vector new_origin=>lidar
    return transformed_scan: message in which datas will be stored
    Homogoneous transformations: https://www.fil.univ-lille.fr/portail/archive21-22/~aubert/m3d/m3d_transformation.pdf
    */
    //RCLCPP_INFO(this->get_logger(), "Transform_data: params : offx='%f'; offy='%f'; off_tetha='%f'",off_vect_x,off_vect_y,off_tetha);
    //RCLCPP_INFO(this->get_logger(), "transform 1");
    sensor_msgs::msg::LaserScan::SharedPtr transformed_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    transformed_scan->ranges.clear();
    int resolution = to_transform_scan->ranges.size();

    // Transform the LaserScan message
    for (int i = 0; i < resolution; ++i) { //initiate
        transformed_scan->ranges.push_back(INFINITY);
    }

    //let's be sure to manipulate angles from 0 to 2pi
    if (off_tetha<0){
        off_tetha=2*M_PI+off_tetha;
    }
    if (off_tetha == 2*M_PI){
        off_tetha=0;
    }
    //translation in homogeneous coordinates
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
    T(0,3) = off_vect_x;
    T(1,3) = off_vect_y;
    //rotation in homogeneous coordinates
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4);
    R(0,0) = cos(off_tetha);
    R(0,1) = -sin(off_tetha);
    R(1,0) = sin(off_tetha);
    R(1,1) = cos(off_tetha);
    //final homogeneous transformation, M1_2 = pass matrix, displacement from new_frame to sensor_frame
    Eigen::MatrixXd M1_2(4, 4);
    M1_2 = T*R;

    //debug_ss << "DEBUG: Pass Matrix:" << std::endl;
    //debug_ss << M1_2 << std::endl;

    //loop variables
    double init_x = 0;
    double init_y = 0;
    double init_val = 0;
    double init_angle = 0;
    double new_x = 0;
    double new_y = 0;
    double new_val = 0;
    double new_angle = 0;
    int new_index = 0;

    for (int ind = 0; ind < resolution; ++ind) { //transform
        init_angle = index_to_angle(ind,resolution);
        init_val = to_transform_scan->ranges[ind];

        //points transformation
        if (!isinf(init_val)){ //if it is infinity, we don't need to compute as the default values in the new scan will be INFINITY
            get_pos(init_x, init_y,init_angle,init_val,0.0,0.0); //position in inital frame, vector: lidar=>point
            Eigen::MatrixXd X2(4, 1); //pos in sensor frame
            X2(0,0) = init_x;
            X2(1,0) = init_y;
            X2(2,0) = 0.0;
            X2(3,0) = 1.0;
            Eigen::MatrixXd X1(4, 1); //pos in newframe
            X1 = M1_2*X2;
            //Unpack
            double new_x2 = X1(0,0);
            double new_y2 = X1(1,0);
            new_angle = atan2(new_y2,new_x2);
            //we want angles from 0 to 2pi
            if (new_angle<0){
                new_angle=2*M_PI+new_angle;
            }
            if (new_angle == 2*M_PI){
                new_angle=0;
            }
            //RCLCPP_INFO(this->get_logger(), "Transform_data: newangle='%f'",new_angle);

            //new norm
            new_val = sqrt(pow(new_x2,2)+pow(new_y2,2));

            new_index = angle_to_index(new_angle,resolution);
            transformed_scan->ranges[new_index] = std::min(static_cast<double>(transformed_scan->ranges[new_index]),new_val); //in case some area are no more accessible from the new origin
            //RCLCPP_INFO(this->get_logger(), "Transform_data: initindex='%d'; initangle='%f'; initval='%f'; initx='%f'; inity='%f'; newx2='%f'; newy2='%f'; newval='%f'; newangle='%f'; newindex='%f'",ind,init_angle,init_val,init_x,init_y,new_x2,new_y2,new_val,new_angle,new_index);
        }

    }
    //RCLCPP_INFO(this->get_logger(), "transform 3");

    //copy result ranges into to_transform_scan
    if(copy_ranges(to_transform_scan,transformed_scan)){ 
        return 1;
    } 
    else{
        debug_ss << "ERROR: filter_360_data: " << "Copy ranges failed" << '\n';
        return 0;
    }
}

int copy_ranges(sensor_msgs::msg::LaserScan::SharedPtr host_scan, sensor_msgs::msg::LaserScan::SharedPtr target_scan){
    try
    {
        int reso = host_scan->ranges.size();
        for (int i = 0; i < reso; ++i) { 
            host_scan->ranges[i] = target_scan->ranges[i];
        }
        return 1;
    }
    catch(const std::exception& e)
    {
        return 0;
    }
}

geometry_msgs::msg::Vector3 adapt_angle(geometry_msgs::msg::Vector3 vect){
    //avoid flip with euleur angles at gimbla lock, work only for z component in that case.
    geometry_msgs::msg::Vector3 new_vect;
    if(abs(abs(vect.x)-M_PI)<0.01 && abs(abs(vect.y)-M_PI)<0.01){
        new_vect.x = 0.0;
        new_vect.y = 0.0;
        new_vect.z = M_PI+vect.z; 
    }
    else{
        new_vect.x = vect.x;
        new_vect.y = vect.y;
        new_vect.z = vect.z;
    }
    return new_vect;
}

std::string vector3ToString(geometry_msgs::msg::Vector3& vector) {
    std::string result = "(" +
        std::to_string(vector.x) + ", " +
        std::to_string(vector.y) + ", " +
        std::to_string(vector.z) + ")";
    return result;
}

geometry_msgs::msg::Vector3 stringToVector3(std::string& str) {
    geometry_msgs::msg::Vector3 vector;
    std::stringstream ss(str);
    // Temporary variables to store parsed values
    double x, y, z;
    // Read values from stringstream
    char delim;
    ss >> delim >> x >> delim >> y >> delim >> z >> delim;
    // Assign parsed values to the vector
    vector.x = x;
    vector.y = y;
    vector.z = z;
    return vector;
}

std::string print_tf(geometry_msgs::msg::TransformStamped transform){
    std::string debug_str;
    std::stringstream debug_ss;
    geometry_msgs::msg::Vector3 euleur_angles = quaternion_to_euler3D(transform.transform.rotation);
    geometry_msgs::msg::Vector3 euleur_angles_head = adapt_angle(euleur_angles);
    debug_ss << "Transform '" << transform.header.frame_id << "' --> '" << transform.child_frame_id
             << "\n    Translation: (" << transform.transform.translation.x << "," << transform.transform.translation.y << "," << transform.transform.translation.z << ") m"
             << "\n    Quaternion: (" << transform.transform.rotation.x << "," << transform.transform.rotation.y << "," << transform.transform.rotation.z <<  "," << transform.transform.rotation.w << ")" 
             << "\n    Euler: (" << rads_to_degrees(euleur_angles.x) << "," << rads_to_degrees(euleur_angles.y) << "," << rads_to_degrees(euleur_angles.z) << ") degs"
             << "\n    Heading_only: " << rads_to_degrees(euleur_angles_head.z)
             << std::endl;
    
    debug_str = debug_ss.str();
    return debug_str;
}


void MatProd_fast4_4(double (&M)[4][4],double (&A)[4][4],double (&B)[4][4]){
    for(int i =0; i< 4; i++){
        for(int j =0; j< 4; j++){
            M[i][j] = 0.0;
            for(int s =0; s< 4; s++){
                M[i][j] += A[i][s]*B[s][j];
            }
        }
    }
}

void MatProd_fast3_3(double (&M)[3][3],double (&A)[3][3],double (&B)[3][3]){
    for(int i =0; i< 3; i++){
        for(int j =0; j< 3; j++){
            M[i][j] = 0.0;
            for(int s =0; s< 3; s++){
                M[i][j] += A[i][s]*B[s][j];
            }
        }
    }
}

void MatProd_fast4_Vect(double (&M)[4][1],double (&A)[4][4],double (&B)[4][1]){
    for(int i =0; i< 4; i++){
        for(int j =0; j< 1; j++){
            M[i][j] = 0.0;
            for(int s =0; s< 4; s++){
                M[i][j] += A[i][s]*B[s][j];
            }
        }
    }
}

void to_identity3_3(double (&mat)[3][3]){
    for(int i =0; i< 3; i++){
        for(int j =0; j< 3; j++){
            if(j==i){
                mat[i][j]=1.0;
            }
            else{
                mat[i][j]=0.0;
            }
        }
    }
}

void to_identity4_4(double (&mat)[4][4]){
    for(int i =0; i< 4; i++){
        for(int j =0; j< 4; j++){
            if(j==i){
                mat[i][j]=1.0;
            }
            else{
                mat[i][j]=0.0;
            }
        }
    }
}

double scalar_projection_fast(double (&a)[3], double (&b)[3]) {
    //projection of 'a' into 'b': https://en.wikipedia.org/wiki/Vector_projection
    // Calculate dot product of a and b
    double dot_product = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    
    // Calculate squared norm of vector b
    double norm_b_squared = pow(b[0],2) + pow(b[1],2) + pow(b[2],2);
    
    // Calculate the projection
    double projection = (dot_product / norm_b_squared);
    
    return projection;
}

double scalar_projection(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    //projection of 'a' into 'b': https://en.wikipedia.org/wiki/Vector_projection
    // Calculate dot product of a and b
    double dot_product = a.dot(b);
    
    // Calculate squared norm of vector b
    double norm_b_squared = b.squaredNorm();
    
    // Calculate the projection
    double projection = (dot_product / norm_b_squared);
    
    return projection;
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> get_4Dmatrix_from_transform(geometry_msgs::msg::TransformStamped transf){
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

    return std::make_tuple(T, R, M1_2);

}

void get_4Dmatrix_from_transform_fast(double (&M)[4][4],geometry_msgs::msg::TransformStamped transf){
    geometry_msgs::msg::Vector3 translation_ = transf.transform.translation;  //translation vector from new_frame to frame_sensor
    geometry_msgs::msg::Vector3 rotation_ = quaternion_to_euler3D(transf.transform.rotation);
    //Translation in homogeneous coordinates
    double T[4][4] = {
                    {1.0,0.0,0.0,translation_.x},
                    {0.0,1.0,0.0,translation_.y},
                    {0.0,0.0,1.0,translation_.z},
                    {0.0,0.0,0.0,1.0}
                    };
    //3D rotation matrix
    double R_temp[3][3];
    rot_matrix_from_euler_fast(R_temp,rotation_);
    //3D rotation in homogeneous coordinates
    double R[4][4];
    to_identity4_4(R);
    for(int i = 0; i<3; i++){
        for(int y = 0; y<3; y++){
            R[i][y] = R_temp[i][y];
        }
    }
    //final homogeneous transformation, M1_2 = pass matrix, displacement from new_frame to init_frame
    double M1_2[4][4];
    MatProd_fast4_4(M1_2,T,R);
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

void rot_matrix_from_euler_fast(double (&R)[3][3], geometry_msgs::msg::Vector3 euler_angles){
    double Rx[3][3];
    to_identity3_3(Rx);
    Rx[1][1] = cos(euler_angles.x);
    Rx[1][2] = -sin(euler_angles.x);
    Rx[2][1] = sin(euler_angles.x);
    Rx[2][2] = cos(euler_angles.x);
    double Ry[3][3];
    to_identity3_3(Ry);
    Ry[0][0] = cos(euler_angles.y);
    Ry[0][2] = sin(euler_angles.y);
    Ry[2][0] = -sin(euler_angles.y);
    Ry[2][2] = cos(euler_angles.y);
    double Rz[3][3];
    to_identity3_3(Rz);
    Rz[0][0] = cos(euler_angles.z);
    Rz[0][1] = -sin(euler_angles.z);
    Rz[1][0] = sin(euler_angles.z);
    Rz[1][1] = cos(euler_angles.z);
    double R_temp[3][3];
    MatProd_fast3_3(R_temp,Ry,Rz);
    MatProd_fast3_3(R,Rx,R_temp);
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
    //debug_ss << "\nAngle_to_index: "<< " mod: " << alpha;
    if (alpha < 0) {
        alpha += 2 * M_PI;
    }
    // Calculate the index
    int ind = static_cast<int>(round((alpha * resolution)/(2*M_PI)));
    //debug_ss <<  " alpha*reso: " << alpha*resolution << " ind: " << ind;
    return ind;
}

double index_to_angle(int ind, int resolution, double elongation){ //default elongation to 2*PI, see function signature
    return (ind*elongation)/resolution;
}

int remap_scan_index(int prev_ind, double prev_angle_start, double prev_angle_end, double prev_reso, double new_angle_start, double new_angle_end, double new_reso){
    int new_ind;
    /*
    return the index in a new scan list.
    */
    double prev_elong = prev_angle_end - prev_angle_start;  
    double new_elong = new_angle_end - new_angle_start;
    double prev_angle_incr = prev_elong/prev_reso;
    double new_angle_incr = new_elong/new_reso;
    int reso_360 = static_cast<int>(round(2*M_PI/prev_angle_incr)); //resoluton that would have prev_list if on a 360deg circle list

   //offset gestion
    double angle_offset = sawtooth(prev_angle_start-new_angle_start);
    if(angle_offset<0){
        angle_offset += 2*M_PI;
    }
    //should have offset between [0,2Pi]
    int ind_offset = angle_to_index(angle_offset,reso_360); //should return index between [0,reso_360]
    new_ind = static_cast<int>(round(fmod(prev_ind + ind_offset,reso_360)));
    //different reso gestion 
    new_ind = static_cast<int>(round((prev_angle_incr*new_ind)/new_angle_incr));
    
    return new_ind;
}

double sawtooth(double x) {
    return std::fmod(x+M_PI,2*M_PI)-M_PI;
}


/*TRASH (need to be tested before use)

geometry_msgs::msg::TransformStamped tf_offset(geometry_msgs::msg::TransformStamped &relative_transform, geometry_msgs::msg::TransformStamped tf1, geometry_msgs::msg::TransformStamped tf2){
    //give transformation from tf1 to tf2

    Eigen::Quaterniond rot_1(tf1.transform.rotation.w,
                            tf1.transform.rotation.x,
                            tf1.transform.rotation.y,
                            tf1.transform.rotation.z);

    Eigen::Quaterniond rot_2(tf2.transform.rotation.w,
                            tf2.transform.rotation.x,
                            tf2.transform.rotation.y,
                            tf2.transform.rotation.z);

    // Perform quaternion subtraction
    Eigen::Quaterniond relative_rotation = rot_2 * rot_1.inverse();

    // Subtract translation components
    double off_x = tf2.transform.translation.x - tf1.transform.translation.x;
    double off_y = tf2.transform.translation.y - tf1.transform.translation.y;
    double off_z = tf2.transform.translation.z - tf1.transform.translation.z;

    // relative_transform represents the difference between the two original transforms
    relative_transform.header = tf2.header;
    relative_transform.transform.rotation.w = relative_rotation.w();
    relative_transform.transform.rotation.x = relative_rotation.x();
    relative_transform.transform.rotation.y = relative_rotation.y();
    relative_transform.transform.rotation.z = relative_rotation.z();
    relative_transform.transform.translation.x = off_x;
    relative_transform.transform.translation.y = off_y;
    relative_transform.transform.translation.z = off_z;
}


void shift_cloud_from_tf(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, geometry_msgs::msg::TransformStamped transf){
    //transf need to be the transformation from new_frame to old_frame
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> Pass_matrixes_new_old = get_4Dmatrix_from_transform(transf);
    //transform each points
    for(int i=0; i<cloud->size(); i++){
        Eigen::MatrixXd Point_old = Eigen::MatrixXd::Identity(4, 1);
        Point_old(0,0) = cloud->points[i].x;
        Point_old(1,0) = cloud->points[i].y;
        Point_old(2,0) = cloud->points[i].z;
        Point_old(3,0) = 1.0;
        
        Eigen::MatrixXd Point_new = std::get<2>(Pass_matrixes_new_old)*Point_old;

        cloud->points[i].x = Point_new(0,0);
        cloud->points[i].y = Point_new(1,0);
        cloud->points[i].z = Point_new(2,0);
    }
}

*/
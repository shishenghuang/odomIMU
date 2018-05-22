

#include <iostream>
#include <ros/ros.h>
#include "odomIMU_msgs/odomIMU.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#define m_PI 3.1415926

using namespace std;
using namespace odomIMU_msgs;
Eigen::Vector3d odom_status; // odom_status[0] is thelta, odom_status[1] is x , odom_status[2] is y
Eigen::Vector2d last_encode;
int recieveData;
const double distance_AB_encode_per_number = m_PI / 14745.6; // degree
const double distance_CAN_encode_per_number =  0.04631 / 1024 ;// m
const int zero_CAN_encode_index = 1307.2; // 
const double L1 = 0.780; // m
const double L2 = 0.2575; // m
const double delta_d = 1.54; // m

std::vector<Eigen::Vector3d> trajectory_list;
void grabMessage(const odomIMU_msgs::odomIMU::ConstPtr& msg);
void grabMessageAndSave(const odomIMU_msgs::odomIMU::ConstPtr& msg);
void eulr2Matrix(const double& thelta, Eigen::Matrix2d& m);

FILE *fp;
FILE *fplog;
int main(int argc, char **argv){

    fp = fopen("data.txt" , "w+");
    fplog = fopen("log.txt" , "w+");
    ros::init(argc, argv, "odomIMU_trajectory");
    ros::start();
	odom_status(0) = 0.0;
    odom_status(1) = 0.0;
    odom_status(2) = 0.0;
    recieveData = 0;

	ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe<odomIMU_msgs::odomIMU>("odomIMU_frame" , 100 , grabMessage);
    //ros::Subscriber odom_sub = nh.subscribe<odomIMU_msgs::odomIMU>("odomIMU_frame" , 100 , grabMessageAndSave);

	ros::Publisher trajectory_pub = nh.advertise<nav_msgs::Path>("odom_trajectory" , 100);

    while(ros::ok()){

        nav_msgs::Path path_list;
        path_list.header.frame_id = "trajectory_frame_info";
        path_list.header.stamp = ros::Time::now();

        int trajectory_num = trajectory_list.size();
        for(int i=0;i<trajectory_num;i++){
            geometry_msgs::PoseStamped pose_i;
            pose_i.pose.orientation.w = 1.0;
            pose_i.pose.orientation.x = 0.0;
            pose_i.pose.orientation.y = 0.0;
            pose_i.pose.orientation.z = 0.0;

            pose_i.pose.position.x = trajectory_list[i](1);
            pose_i.pose.position.y = trajectory_list[i](2);
            pose_i.pose.position.z = 0.0;
            path_list.poses.push_back(pose_i);
        }
        trajectory_pub.publish(path_list);
        ros::spinOnce();
    }
    ros::shutdown();

    return 0;
}

void eulr2Matrix(const double& thelta, Eigen::Matrix2d& m){

    m(0,0) = cos(thelta);
    m(0,1) = -sin(thelta);
    m(1,0) = sin(thelta);
    m(1,1) = cos(thelta);
}

void grabMessageAndSave(const odomIMU_msgs::odomIMU::ConstPtr& msg){

    int64_t time_ = (int64_t)(msg->header.stamp.sec) * 1000000000 + msg->header.stamp.nsec ;
    int encode0 = msg->encode0Cnt;
    int encode1 = msg->encode1Cnt;
    fprintf(fp , "%lld %d %d\n" , time_ , encode0 , encode1);
}

void grabMessage(const odomIMU_msgs::odomIMU::ConstPtr& msg){

    if(!recieveData){
        trajectory_list.push_back(odom_status);
        last_encode(0) = msg->encode0Cnt;
        last_encode(1) = msg->encode1Cnt; 
        std::cout << "recieveData: " << last_encode(0) << ", " << last_encode(1) << std::endl; 
        recieveData = 1;
        return;      
    }

    Eigen::Vector2d current_encode;
    current_encode(0) = msg->encode0Cnt;
    current_encode(1) = msg->encode1Cnt;

   std::cout << "recieveData: " << last_encode(0) << ", " << last_encode(1) << std::endl; 
    fprintf(fplog , "recieveData: %d %d\n" , last_encode(0) , last_encode(1));

    double thelta = 0.5 * ( last_encode(1) + current_encode(1) );
    double d = 0;
    double d1 = std::max(last_encode(0) , current_encode(0)) - std::min(last_encode(0) , current_encode(0));
    double d2 = 1024 - d1;
    int isGap = 0;
    if(d1 > d2){
        // oh, d = d2, isGap = 0;
        d = d2;
        isGap = 0;
    }
    else{
        // en, d = d1, isGap = 1;
        d = d1; 
        isGap = 1;
    }

    if(isGap = 0){
        // in the gap condition
        // if current_encode(0) < last_encode(0), the motion is forward
        // otherwise, the motion is backward;
        if(current_encode(0) < last_encode(0)){
            d = d; // forward
        }
        else{
            d = -1.0 * d ; // backward, d is a negtive number 
        }
    }
    else{
        // in the non gap condition
        // if current_encode(0) > last_encode(0), the motion is forward
        // otherwise, the motion is backward
            if(current_encode(0) > last_encode(0)){
            d = d; // forward
        }
        else{
            d = -1.0 * d ; // backward, d is a negtive number 
        }    
    }

    double thelta_angle = (thelta - zero_CAN_encode_index) * distance_AB_encode_per_number;
    double rad_d = d * distance_CAN_encode_per_number;
    std::cout << "thelta_angle = " << thelta_angle << ", rad_d = " << rad_d << std::endl;
    fprintf(fplog , "thelta_angle = %lf rad_d = %lf\n" , thelta_angle , rad_d);
    Eigen::Vector3d current_odom_status;

    Eigen::Matrix2d R_o_w;
    eulr2Matrix(odom_status(0) , R_o_w);
    Eigen::Matrix2d R_w_o = R_o_w.inverse();    
    
    if(fabs(thelta_angle) <= 0.001){
        Eigen::Vector2d dx = R_w_o * Eigen::Vector2d(0 , rad_d);
        std::cout << " dx = " << dx(0) << ", " << dx(1) << std::endl;
        fprintf(fplog , "dx = %lf , dy = %lf\n" , dx(0), dx(1));
        if(rad_d == 0){
            current_odom_status(0) = odom_status(0);
        }
        else if(rad_d > 0) {
            current_odom_status(0) = odom_status(0);// + thelta_angle;
        }
        else{
            current_odom_status(0) = odom_status(0);// - thelta_angle;
        }
        //current_odom_status(0) = odom_status(0) + (rad_d > 0) ? thelta_angle : (-1.0 * thelta_angle);
        current_odom_status(1) = odom_status(1) + dx(0);
        current_odom_status(2) = odom_status(2) + dx(1);
    }
    else{

        if(thelta_angle > 0 && rad_d > 0){
            double R1 = fabs(delta_d / tan(thelta_angle));
            double phi = fabs(rad_d / delta_d * cos(thelta_angle));
            Eigen::Vector2d dx = 2 * R1 * R_w_o * Eigen::Vector2d(sin(phi/2)*sin(phi/2) , cos(phi/2)*sin(phi/2));
            current_odom_status(0) = odom_status(0) + phi;
            current_odom_status(1) = odom_status(1) + dx(0);
            current_odom_status(2) = odom_status(2) + dx(1);
            std::cout << " dx = " << dx(0) << ", " << dx(1) << std::endl;
            fprintf(fplog , "dx = %lf , dy = %lf\n" , dx(0), dx(1));            
        }
        else if(thelta_angle > 0 && rad_d < 0){
            double R1 = fabs(delta_d / tan(thelta_angle));
            double phi = fabs(rad_d / delta_d * cos(thelta_angle));
            Eigen::Vector2d dx = 2 * R1 * R_w_o * Eigen::Vector2d(sin(phi/2)*sin(phi/2) , -cos(phi/2)*sin(phi/2));
            current_odom_status(0) = odom_status(0) - phi;
            current_odom_status(1) = odom_status(1) + dx(0);
            current_odom_status(2) = odom_status(2) + dx(1);
            std::cout << " dx = " << dx(0) << ", " << dx(1) << std::endl;
            fprintf(fplog , "dx = %lf , dy = %lf\n" , dx(0), dx(1));            
        }
        else if(thelta_angle < 0 && rad_d > 0){
            double R1 = fabs(delta_d / tan(thelta_angle));
            double phi = fabs(rad_d / delta_d * cos(thelta_angle));
            Eigen::Vector2d dx = 2 * R1 * R_w_o * Eigen::Vector2d(-sin(phi/2)*sin(phi/2) , cos(phi/2)*sin(phi/2));
            current_odom_status(0) = odom_status(0) - phi;
            current_odom_status(1) = odom_status(1) + dx(0);
            current_odom_status(2) = odom_status(2) + dx(1);
            std::cout << " dx = " << dx(0) << ", " << dx(1) << std::endl;
            fprintf(fplog , "dx = %lf , dy = %lf\n" , dx(0), dx(1));
        }
        else{
            double R1 = fabs(delta_d / tan(thelta_angle));
            double phi = fabs(rad_d / delta_d * cos(thelta_angle));
            Eigen::Vector2d dx = 2 * R1 * R_w_o * Eigen::Vector2d(-sin(phi/2)*sin(phi/2) , -cos(phi/2)*sin(phi/2));
            current_odom_status(0) = odom_status(0) + phi;
            current_odom_status(1) = odom_status(1) + dx(0);
            current_odom_status(2) = odom_status(2) + dx(1);
            std::cout << " dx = " << dx(0) << ", " << dx(1) << std::endl;
            fprintf(fplog , "dx = %lf , dy = %lf\n" , dx(0), dx(1));
        }
    }

    trajectory_list.push_back(current_odom_status);

    odom_status(0) = current_odom_status(0);
    odom_status(1) = current_odom_status(1);
    odom_status(2) = current_odom_status(2);
    last_encode(0) = current_encode(0);
    last_encode(1) = current_encode(1);
    std::cout << "current postion: " << odom_status(0) << ", " << odom_status(1) << ", " << odom_status(2) << std::endl;
    fprintf(fplog , "current position: %lf %lf %lf\n" , odom_status(0) , odom_status(1) , odom_status(2));
}

/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "ros/ros.h"
#include "globalOpt.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <queue>
#include <mutex>
#include <std_msgs/Bool.h>

struct Point
{
    double x;
    double y;
    double z;
};

int nFail = 0;

GlobalOptimization globalEstimator;
ros::Publisher pub_global_odometry, pub_global_path, pub_car, marker_pub, reset_pub;
nav_msgs::Path *global_path;
double last_vio_t = -1;
std::queue<sensor_msgs::NavSatFixConstPtr> gpsQueue;
std::mutex m_buf;
std::vector<Point> vec_point;

void publish_car_model(double t, Eigen::Vector3d t_w_car, Eigen::Quaterniond q_w_car)
{
    visualization_msgs::MarkerArray markerArray_msg;
    visualization_msgs::Marker car_mesh;
    car_mesh.header.stamp = ros::Time(t);
    car_mesh.header.frame_id = "world";
    car_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    car_mesh.action = visualization_msgs::Marker::ADD;
    car_mesh.id = 0;

    car_mesh.mesh_resource = "package://global_fusion/models/car.dae";

    Eigen::Matrix3d rot;
    rot << 0, 0, -1, 0, -1, 0, -1, 0, 0;
    
    Eigen::Quaterniond Q;
    Q = q_w_car * rot; 
    car_mesh.pose.position.x    = t_w_car.x();
    car_mesh.pose.position.y    = t_w_car.y();
    car_mesh.pose.position.z    = t_w_car.z();
    car_mesh.pose.orientation.w = Q.w();
    car_mesh.pose.orientation.x = Q.x();
    car_mesh.pose.orientation.y = Q.y();
    car_mesh.pose.orientation.z = Q.z();

    car_mesh.color.a = 1.0;
    car_mesh.color.r = 1.0;
    car_mesh.color.g = 0.0;
    car_mesh.color.b = 0.0;

    float major_scale = 2.0;

    car_mesh.scale.x = major_scale;
    car_mesh.scale.y = major_scale;
    car_mesh.scale.z = major_scale;
    markerArray_msg.markers.push_back(car_mesh);
    pub_car.publish(markerArray_msg);
}

void GPS_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg)
{
    //printf("gps_callback! \n");
    m_buf.lock();
    gpsQueue.push(GPS_msg);
    m_buf.unlock();
}

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //printf("vio_callback! \n");
    double t = pose_msg->header.stamp.toSec();
    last_vio_t = t;
    Eigen::Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Eigen::Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;
    globalEstimator.inputOdom(t, vio_t, vio_q);

    double xyz[3] = {0};

    m_buf.lock();
    while(!gpsQueue.empty())
    {
        sensor_msgs::NavSatFixConstPtr GPS_msg = gpsQueue.front();
        double gps_t = GPS_msg->header.stamp.toSec();
        //printf("vio t: %f, gps t: %f \n", t, gps_t);
        // 10ms sync tolerance
        if(gps_t >= t - 0.03 && gps_t <= t + 0.03)
        {
            //printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
            double latitude = GPS_msg->latitude;
            double longitude = GPS_msg->longitude;
            double altitude = 0.0;
            //int numSats = GPS_msg->status.service;
            double pos_accuracy = GPS_msg->position_covariance[0];
            if(pos_accuracy <= 0)
                pos_accuracy = 1;
            //printf("receive covariance %lf \n", pos_accuracy);
            //if(GPS_msg->status.status > 8)
                globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy);
            gpsQueue.pop();
            ///
            globalEstimator.GPS2XYZ(latitude, longitude, altitude, xyz);
            Point p;
            p.x = xyz[0];
            p.y = xyz[1];
            p.z = xyz[2];
            //cout<<p.x<<p.y<<p.z<<endl;
            vec_point.push_back(p);
            
            visualization_msgs::MarkerArray node_arr;
            //for (size_t i = 0; i < vec_point.size(); i++)
            //{
              Point o_node = vec_point[vec_point.size()-1];

              visualization_msgs::Marker node;
              node.header.frame_id = "world"; // map frame 기준
              node.header.stamp = ros::Time::now();
              node.type = visualization_msgs::Marker::SPHERE;
              node.id = vec_point.size();
              node.action = visualization_msgs::Marker::ADD;
              node.pose.orientation.w = 1.0;
              node.pose.position.x =  o_node.x; //노드의 x 좌표
              node.pose.position.y =  o_node.y; //노드의 y 좌표
              // Points are green
              node.color.g = 0.5;
              node.color.a = 1.0;
              node.scale.x = 1.0;
              node.scale.y = 1.0;     
              node_arr.markers.push_back(node);
            //}
            marker_pub.publish(node_arr);
            ///
            break;
        }
        else if(gps_t < t - 0.03)
            gpsQueue.pop();
        else if(gps_t > t + 0.03)
            break;
    }
    m_buf.unlock();

    Eigen::Vector3d global_t;
    Eigen:: Quaterniond global_q;
    globalEstimator.getGlobalOdom(global_t, global_q);

    nav_msgs::Odometry odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = global_t.x();
    odometry.pose.pose.position.y = global_t.y();
    odometry.pose.pose.position.z = global_t.z();
    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    odometry.pose.pose.orientation.w = global_q.w();
    pub_global_odometry.publish(odometry);
    double res = sqrt(pow(xyz[0]-odometry.pose.pose.position.x, 2)+pow(xyz[1]-odometry.pose.pose.position.y, 2)+pow(xyz[2]-odometry.pose.pose.position.z, 2));
    //cout<< "res : " << res <<endl;
    if(res>50)
      nFail++;
    else if(nFail>0)
      nFail--;      
    if(nFail>3)
    {
      printf("fail\n");
      nFail = 0;
      std_msgs::Bool failMsg;
      failMsg.data = 1;
      reset_pub.publish(failMsg);
      globalEstimator.localPoseMap.clear();
      globalEstimator.globalPoseMap.clear();
      globalEstimator.GPSPositionMap.clear();
    }
    pub_global_path.publish(*global_path);
    //publish_car_model(t, global_t, global_q);


    // write result to file
    std::ofstream foutC("/home/tony-ws1/output/vio_global.csv", ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(0);
    foutC << pose_msg->header.stamp.toSec() * 1e9 << ",";
    foutC.precision(5);
    foutC << global_t.x() << ","
            << global_t.y() << ","
            << global_t.z() << ","
            << global_q.w() << ","
            << global_q.x() << ","
            << global_q.y() << ","
            << global_q.z() << endl;
    foutC.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "globalEstimator");
    ros::NodeHandle n("~");

    global_path = &globalEstimator.global_path;

    ros::Subscriber sub_GPS = n.subscribe("/gps", 100, GPS_callback);
    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 100, vio_callback);
    pub_global_path = n.advertise<nav_msgs::Path>("global_path", 100);
    pub_global_odometry = n.advertise<nav_msgs::Odometry>("global_odometry", 100);
    pub_car = n.advertise<visualization_msgs::MarkerArray>("car_model", 1000);
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("marker/node", 1000);
    reset_pub = n.advertise<std_msgs::Bool>("fail", 100);
    
    ros::spin();
    return 0;
}

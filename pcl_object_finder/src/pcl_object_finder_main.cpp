// Created by Su Lu on Nov, 2015.
// Copyright @ 2015 Su Lu. All rights reserved.
#include <ros/ros.h>
#include <pcl_object_finder/pcl_object_finder.h>
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

int main(int argc, char** argv) {
    // Do some initialization here
    ros::init(argc, argv, "pcl_object_finder");
    ros::NodeHandle nh;
    ros::Rate rate(2);

    PclObjectFinder pclObjectFinder(&nh);
    cout<<"warming up callbacks..."<<endl;
    for (int i=0;i<100;i++) {
        ros::spinOnce();
        //cout<<"spin "<<i<<endl;
        ros::Duration(0.01).sleep();
    }

    Eigen::MatrixXd selected_points_mat;
    pclObjectFinder.returnSelectedPointCloud(selected_points_mat);

    Eigen::Vector3d plane_normal;
    double plane_dist = 0.0;
    pclObjectFinder.fitPointsToPlane(&selected_points_mat, plane_normal, plane_dist);

    Eigen::Vector3d centroid_vec;
    centroid_vec = pclObjectFinder.findCentroid(&points_mat);
    
    std::vector<Eigen::Vector3d> points_vec;
    pclObjectFinder.findPointsOnPlane(points_vec, centroid_vec, plane_dis);
    
    
    return 0;
}
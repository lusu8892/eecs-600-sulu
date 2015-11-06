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

    double 
    Eigen::MatrixXd* points_mat;
    pclObjectFinder.returnSelectedPointCloud(points_mat);


    return 0;
}
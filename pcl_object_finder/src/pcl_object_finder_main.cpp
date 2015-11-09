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

    //set up a publisher to display clouds in rviz:
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_cloud_display", 1);
    //pcl::PointCloud<pcl::PointXYZ> & outputCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr display_cloud; // instantiate a pointcloud object, which will be used for display in rviz
    sensor_msgs::PointCloud2 pcl2_display_cloud; //(new sensor_msgs::PointCloud2); //corresponding data type for ROS message


    Eigen::MatrixXf selected_points_mat;
    pclObjectFinder.returnSelectedPointCloud(selected_points_mat);

    Eigen::Vector3f plane_normal;
    double plane_dist = 0.0;
    pclObjectFinder.fitPointsToPlane(&selected_points_mat, plane_normal, plane_dist);

    Eigen::Vector3f centroid_vec;
    centroid_vec = pclObjectFinder.findCentroid(&points_mat);
    
    std::vector<Eigen::Vector3d> points_vec;
    pclObjectFinder.findPointsOnPlane(display_cloud, centroid_vec, plane_dis);
    
    pcl::toROSMsg(*display_cloud, pcl2_display_cloud); //convert datatype to compatible ROS message type for publication
    pcl2_display_cloud.header.stamp = ros::Time::now(); //update the time stamp, so rviz does not complain        
    pubCloud.publish(pcl2_display_cloud); //publish a point cloud that can be viewed in rviz (under topic pcl_cloud_display)

    ros::Duration(0.5).sleep(); // sleep for half a second
    ros::spinOnce();

    return 0;
}
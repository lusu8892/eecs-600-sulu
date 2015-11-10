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
    ros::NodeHandle nodehandle;
    ros::Rate rate(2);

    PclObjectFinder pclObjectFinder(&nh);

    while (!pclObjectFinder.gotKinectCloud())
    {
        ROS_INFO("did not receive pointcloud");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("got a pointcloud");

    //set up a publisher to display clouds in rviz:
    ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_cloud_display", 1);
    //pcl::PointCloud<pcl::PointXYZ> & outputCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr display_cloud; // instantiate a pointcloud object, which will be used for display in rviz
    sensor_msgs::PointCloud2 pcl2_display_cloud; //(new sensor_msgs::PointCloud2); //corresponding data type for ROS message
    while(ros::ok())
    {
        if (pclObjectFinder.gotSelectedPoints())
        {
            Eigen::MatrixXf selected_points_mat;
            pclObjectFinder.returnSelectedPointCloud(selected_points_mat);

            pclObjectFinder.resetGotSelectedPoints();

            Eigen::Vector3f plane_normal;
            double plane_dist = 0.0;
            pclObjectFinder.fitPointsToPlane(&selected_points_mat, plane_normal, plane_dist);

            Eigen::Vector3f centroid_vec;
            centroid_vec = pclObjectFinder.findCentroid(&points_mat);

            std::vector<Eigen::Vector3d> points_vec;
            pclObjectFinder.findPointsOnPlane(display_cloud, centroid_vec, plane_dis);

            pcl::toROSMsg(*display_cloud, pcl2_display_cloud); //convert datatype to compatible ROS message type for publication
            pcl2_display_cloud.header.stamp = ros::Time::now(); //update the time stamp, so rviz does not complain        
            pub_cloud.publish(pcl2_display_cloud); //publish a point cloud that can be viewed in rviz (under topic pcl_cloud_display)

            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
        // to let selectCB call function get selected Pcl data
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    return 0;
}
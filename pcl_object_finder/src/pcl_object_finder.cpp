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


PclObjectFinder::PclObjectFinder(ros::Nodelhandle* nodehandle): nh_(*nodehandle)
{
    // before initializing other member function, the pointer type variables need to be initialized at first
    pclKinect_ptr_ = new PointCloud<pcl::PointXYZ>;
    pclTransformed_ptr_ = new PointCloud<pcl::PointXYZ>;
    pclSelectedPoints_ptr_ = new PointCloud<pcl::PointXYZ>;
    pclTransformedSelectedPoints_ptr_ = new PointCloud<pcl::PointXYZ>;

    initializeSubscribers();
    initializePublishers();
    got_kinect_cloud_=false;
    got_selected_points_=false;

}
PclObjectFinder::~PclObjectFinder()
{
    delete pclKinect_ptr_;
    delete pclTransformed_ptr_;
    delete pclSelectedPoints_ptr_;
    delete pclTransformedSelectedPoints_ptr_;
}

PclObjectFinder::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");

    pointcloud_subscriber_ = nh_.subscribe("/kinect/depth/points", 1, &PclObjectFinder::kinectCB, this);
    // add more subscribers here, as needed

    // subscribe to "selected_points", which is published by Rviz tool
    selected_points_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2> ("/selected_points", 1,
        &PclObjectFinder::selectCB, this);
}

PclObjectFinder::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("object_recognized_pointcloud", 1, true);
    //add more publishers, as needed
}

void PclObjectFinder::kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    ROS_INFO("callback from kinect pointcloud pub");
    // convert/copy the cloud only if desired
    if (!got_kinect_cloud_)
    {
        pcl::fromROSMsg(*cloud, *pclKinect_ptr_);
        ROS_INFO("kinectCB: got cloud with %d * %d points", (int) pclKinect_ptr_ -> width, (int) pclKinect_ptr_ -> height);
        got_kinect_cloud_ = true; //cue to "main" that callback received and saved a pointcloud        
    }
    pcl::io::savePCDFileASCII ("snapshot.pcd", *pclKinect_ptr_);
    ROS_INFO("saved PCD image consisting of %d data points to snapshot.pcd", (int) pclKinect_ptr_ -> points.size());
}

// this callback wakes up when a new "selected Points" message arrives
void PclObjectFinder::selectCB(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    pcl::fromROSMsg(*cloud, *pclSelectedPoints_ptr_);
    ROS_INFO("RECEIVED NEW PATCH w/  %d * %d points", pclSelectedPoints_ptr_->width, pclSelectedPoints_ptr_->height);
    got_selected_points_ = true;
}


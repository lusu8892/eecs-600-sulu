// pcl_object_finder.h header file; doxygen comments follow //
/// sulu; Nov, 2015
/// Include this file in "pcl_object_finder.cpp", and in any main that uses this library/
/// This class offers the functions to accepts user input of "selected points" from Rviz,
/// then, finds points that are coplanar with the selected patch,
/// after that displays a computed pointcloud, overlayed on the Rviz display 
/// showing all of the Kinect points that are co-planar with your selected patch.

#ifndef PCL_OBJECT_FINDER_H_
#define PCL_OBJECT_FINDER_H_
#include <ros/ros.h> //generic C++ stuff
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <vector>

#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <sensor_msgs/PointCloud2.h> //useful ROS message types
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/transform_listener.h>  // transform listener headers
#include <tf/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>  //point-cloud library headers; likely don't need all these
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>

class PclObjectFinder
{
public:
    PclObjectFinder(ros::Nodelhandle* nodehandle)
    ~PclObjectFinder();
    Eigen::Affine3f transformTFToEigen(const tf::Transform &t);
    void transformCloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::Affine3f A,
        PointCloud<pcl::PointXYZ>::Ptr outputCloud);
    void fitPointsToPlane(Eigen::MatrixXd points_mat,Eigen::Vector3d &plane_normal, double &plane_dist);
    void showObjectSurface(PointCloud<pcl::PointXYZ>::Ptr outputCloud, PointCloud<pcl::PointXYZ>::Ptr pointFound);

private:
    ros::NodeHandle nh_;
    // some objects to support subscriber, service, and publisher
    ros::Subscriber pointcloud_subscriber_; // use this to subscribe to a pointcloud topic
    ros::Subscriber selected_points_subscriber_; // this to subscribe to "selectedPoints" topic from Rviz
    
    //ros::ServiceServer minimal_service_; //maybe want these later
    ros::Publisher  pointcloud_publisher_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclKinect_ptr_; // (new PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclTransformed_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclSelectedPoints_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclTransformedSelectedPoints_ptr_;
    
    bool got_kinect_cloud_;
    bool got_selected_points_;
    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    //void initializeServices();

    void selectCB(const sensor_msgs::PointCloud2ConstPtr& cloud);
    void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud);
};
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

const double R_GAZEBO_BEER = 0.06; //estimated from ruler tool...example to fit a cylinder of this radius to data
const double R_GAZEBO_BEER_TOL = 0.01;
const double H_GAZEBO_BEER_TOL = 0.001;

class PclObjectFinder
{
public:
    PclObjectFinder(ros::Nodelhandle* nodehandle)
    ~PclObjectFinder();
    void returnSelectedPointCloud(Eigen::MatrixXd& points_mat)
    Eigen::Vector3d findCentroid(Eigen::MatrixXd* points_mat);

    void resetGotSelectedPoints() {got_selected_points_= false;};
    bool gotSselectedPoints() {return got_selected_points_;};
    void fitPointsToPlane(Eigen::MatrixXd* points_mat, Eigen::Vector3d &plane_normal, double &plane_dist);

    void findPointsOnPlane(std::vector<Eigen::Vector3d>& points_vec_temp, Eigen::Vector3d centroid_vec, double plane_dist);

private:
    ros::NodeHandle nh_;
    // some objects to support subscriber, service, and publisher
    ros::Subscriber pointcloud_subscriber_; // use this to subscribe to a pointcloud topic
    ros::Subscriber selected_points_subscriber_; // this to subscribe to "selectedPoints" topic from Rviz
    
    //ros::ServiceServer minimal_service_; //maybe want these later
    ros::Publisher  pointcloud_publisher_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclKinect_ptr_; // (new PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclTransformedKinect_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclSelectedPoints_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclTransformedSelectedPoints_ptr_;

    tf::StampedTransform tf_sensor_frame_to_torso_frame_;
    tf::TransformListener tf_listener_;

    bool got_kinect_cloud_;
    bool got_selected_points_;
    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    //void initializeServices();

    void kinectCB(const sensor_msgs::PointCloud2ConstPtr& kinectCloud);
    void selectCB(const sensor_msgs::PointCloud2ConstPtr& selectedCloud);
    Eigen::Affine3f transformTFToEigen(const tf::Transform &t);
    void transformCloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::Affine3f A,
        PointCloud<pcl::PointXYZ>::Ptr outputCloud);
    void transformPointCloudWrtTorso(PointCloud<pcl::PointXYZ>::Ptr inputCloud, PointCloud<pcl::PointXYZ>::Ptr cloud_transformed);
    void convertPclToEigen(PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::MatrixXd* points_mat);
    void convertEigenToPcl(Eigen::MatrixXd* points_mat, PointCloud<pcl::PointXYZ>::Ptr outCloud);
    void PclObjectFinder::convertEigenToPcl(std::vector<Eigen::Vector3d>* eigen_to_pcl_vec, PointCloud<pcl::PointXYZ>::Ptr outputCloud);
    // double distBtwPoints(double x_0, double y_0, double x, double y);
};
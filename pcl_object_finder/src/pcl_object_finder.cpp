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
    pclTransformedKinect_ptr_ = new PointCloud<pcl::PointXYZ>;
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
    delete pclTransformedKinect_ptr_;
    delete pclSelectedPoints_ptr_;
    delete pclTransformedSelectedPoints_ptr_;
}

void PclObjectFinder::returnSelectedPointCloud(Eigen::MatrixXd& points_mat)
{
    // transform the point cloud data acquired from selectCB to point cloud data wrt torso frame
    void transformPointCloudWrtTorso(pclKinect_ptr_, pclTransformedSelectedPoints_ptr_);
    // convert point cloud data to eigen type
    convertPclToEigen(pclTransformedSelectedPoints_ptr_, &points_mat);
}

Eigen::Vector3d PclObjectFinder::findCentroid(Eigen::MatrixXd* points_mat)
{
    // first compute the centroid of the selected point cloud data
    Eigen::Vector3d centroid_vec;
    // initializing the element in centroid vector all as zero
    centroid_vec = Eigen::MatrixXd::Zero(3,1);
    // add all the selected point cloud data (points) together
    int npts = *points_mat.cols();
    for (int i = 0; i < npts; ++i)
    {
        centroid_vec += *points_mat.cols(i);
    }
    // divided the sum matrix by the number of points to get centroid
    centroid_vec /= npts;
    return centroid_vec;
}

void PclObjectFinder::fitPointsToPlane(Eigen::MatrixXd* points_mat,
        Eigen::Vector3d &plane_normal, double &plane_dist);
{
    // first compute the centroid of the selected point cloud data
    Eigen::Vector3d centroid_vec;
    centroid_vec = findCentroid(points_mat);

    // subtract the centroid from all points in points_mat;
    Eigen::MatrixXd points_offset_mat = *points_mat;
    for (int i = 0; i < npts; ++i)
    {
        points_offset_mat.cols(i) = points_offset_mat.cols(i) - centroid_vec;
    }
    // compute the covariance matrix
    Eigen::Matrix3d covar_mat;
    covar_mat = points_offset_mat * points_offset_mat.transpose();

    // caompute the eigenvalue and eigenvector of the covariance matrix
    Eigen::EigenSolver<Eigen::Matrix3d> es3f(covar_mat);
    // get the real part of the eigen value
    Eigen::VectorXd evals;
    evals = es3f.eigenvalue().real();

    // Eigen::Vector3cf complex_vec;
    // complex_vec = es3f.eigenvector().cols(0);
    // plane_normal = complex_vec;
    // find the most smallest eigenvalue and its corresponding eigenvector
    double min_lambda = evals[0];
    int i_min = 0;
    for (int i = 0; i < 3; ++i)
    {
        if (evals[i] < min_lambda)
        {
            min_lambda = evals[i];
            i_min = i;
            plane_normal = es3f.eigenvector().cols(i_min).real();
        }
    }
    plane_dist = plane_normal.dot(centroid);
}

void PclObjectFinder::findPointsOnPlane(Eigen::MatrixXd& points_mat,
            Eigen::Vector3d centroid_vec, double plane_dist)
{
    Eigen::MatrixXd kinectCB_points_mat;
    // transform the point cloud data acquired from kinectCB to point cloud data wrt torso frame
    void transformPointCloudWrtTorso(pclKinect_ptr_, pclTransformedKinect_ptr_);
    // convert point cloud data to eigen type
    convertPclToEigen(pclTransformedKinect_ptr_, &kinectCB_points_mat);
    
    
}

void PclObjectFinder::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");

    pointcloud_subscriber_ = nh_.subscribe("/kinect/depth/points", 1, &PclObjectFinder::kinectCB, this);
    // add more subscribers here, as needed

    // subscribe to "selected_points", which is published by Rviz tool
    selected_points_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2> ("/selected_points", 1,
        &PclObjectFinder::selectCB, this);
}

void PclObjectFinder::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("object_recognized_pointcloud", 1, true);
    //add more publishers, as needed
}

void PclObjectFinder::kinectCB(const sensor_msgs::PointCloud2ConstPtr& kinectCloud)
{
    ROS_INFO("callback from kinect pointcloud pub");
    // convert/copy the cloud only if desired
    if (!got_kinect_cloud_)
    {
        // convert sensor_msgs type point cloud data to pcl type point cloud
        pcl::fromROSMsg(*kinectCloud, *pclKinect_ptr_);
        ROS_INFO("kinectCB: got cloud with %d * %d points", (int) pclKinect_ptr_ -> width,
            (int) pclKinect_ptr_ -> height);
        got_kinect_cloud_ = true; //cue to "main" that callback received and saved a pointcloud
    }
    // pcl::io::savePCDFileASCII ("snapshot.pcd", *pclKinect_ptr_);
    // ROS_INFO("saved PCD image consisting of %d data points to snapshot.pcd",
    //     (int) pclKinect_ptr_ -> points.size());
}

// this callback wakes up when a new "selected Points" message arrives
void PclObjectFinder::selectCB(const sensor_msgs::PointCloud2ConstPtr& selectedCloud)
{
    ROS_INFO("callback from selected pointcloud pub");
    if (!got_selected_points_)
    {
        // convert sensor_msgs type point cloud data to pcl type point cloud
        pcd::fromROSMsg(*selectedCloud, *pclSelectedPoints_ptr_);
        ROS_INFO("selectCB: got cloud with %d * %d points", (int) pclSelectedPoints_ptr_ -> width,
            (int) pclSelectedPoints_ptr_ -> height);
        got_selected_points_ = true;
    }
}

Eigen::Affine3f PclObjectFinder::transformTFToEigen(const tf::Transform &t)
{
    Eigen::Affine3f e;  // treat the Eigen::Affine as a 4x4 matrix

    // filling in the first three rows elements
    for (int row = 0; row < 3; ++row)
    {
        e.matrix()(row,3) = t.getOrigin()[row];  // filling in the 4th column elements
        for (int col = 0; col < 3; ++col)
        {
            e.matrix()(row, col) = t.getBasis()[row][col];  // filling in first three columns elements
        }
    }
    // filling in 4th row elements
    for (int col = 0; col < 3; ++col)
    {
        e.matrix()(3, col) = 0;
    }
    // filling in the (4,4) element
    e.matrix()(3,3) = 1;
    return e;
}

void PclObjectFinder::transformCloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::Affine3f A,
        PointCloud<pcl::PointXYZ>::Ptr outputCloud)
{
    outputCloud -> header = inputCloud -> header;
    outputCloud -> is_dense = inputCloud -> is_dense;
    outputCloud -> width = inputCloud -> width;
    outputCloud -> height = inputCloud -> height;
    int npts = inputCloud -> points.size();
    outputCloud -> points.resize();

    // getVector3fMap() READING points from a pointCloud, with conversions to Eigen compatible data
    // Or allowing to WRITE points to a pointcloud, with conversions from Eigen type data.
    for (int i = 0; i < npts; ++ i)
    {
        outputCloud ->points[i].getVector3fMap() = A * inputCloud -> points[i]. getVector3fMap();
    }
}

void PclObjectFinder::transformPointCloudWrtTorso(PointCloud<pcl::PointXYZ>::Ptr inputCloud,
        PointCloud<pcl::PointXYZ>::Ptr cloud_transformed)
{
    Eigen::Affine3f A;  // Affine type transformation matrix

    // holder for processed point clouds
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);

    // get the tranformation matrix
    ROS_INFO("get current transform from sensor frame to torso frame: ");
    tf_listener_.lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame_);

    // covert transformation matrix to Affine type
    ROS_INFO("convert to Affine: ");
    A = transformTFToEigen(tf_sensor_frame_to_torso_frame_);
    ROS_INFO("resulting Affine: rotation, translation");
    ROS_INFO_STREAM("the orientation" << A.linear());
    ROS_INFO_STREAM("the translation" << A.translation().transpose());

    // transfer sensor view of point cloud to torso view of point cloud
    transformCloud(inputCloud, A, cloud_transformed);
    ROS_INFO("saving transformed cloud as snapshot_wrt_torso.pcd");

    // save tranformed point cloud to local disk for offline process
    pcl::io::savePCDFileASCII("snapshot_wrt_torso", *cloud_transformed);
}

void PclObjectFinder::convertPclToEigen(PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::MatrixXd* pcl_to_eigen_mat);
{
    Eigen::MatrixXf pcl_to_eigen_matf;
    int npts = inputCloud -> points.size();
    for (int i = 0; i < npts; ++i)
    {
        pcl_to_eigen_matf.cols(i) = inputCloud -> points[i].getVector3fMap();
    }
    *pcl_to_eigen_mat = pcl_to_eigen_matf.cast<float>();
}


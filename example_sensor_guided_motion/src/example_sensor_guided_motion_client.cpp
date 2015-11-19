// largely borrowed from example_baxter_cart_move_action_client: 
// wsn, Nov, 2015
// illustrates use of baxter_cart_move_as action server called "cartMoveActionServer" together
// with cwru_pcl_utils, borrowing from cwru_pcl_utils_example_main.cpp

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cwru_action/cwru_baxter_cart_moveAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cwru_pcl_utils/cwru_pcl_utils.h>

//define a class to encapsulate some of the tedium of populating and sending goals,
// and interpreting responses; 
// Should make this a library, instead of copying/pasting from example_baxter_cart_move_action_client
#include "arm_motion_commander_class.cpp" // bring in class definition, verbatim



int main(int argc, char** argv) {
    ros::init(argc, argv, "example_cart_move_action_client"); // name this node 
    ros::NodeHandle nh; //standard ros node handle     
    ArmMotionCommander arm_motion_commander(&nh);
    CwruPclUtils cwru_pcl_utils(&nh);
    while (!cwru_pcl_utils.got_kinect_cloud()) {
        ROS_INFO("did not receive pointcloud");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("got a pointcloud");
    tf::StampedTransform tf_sensor_frame_to_torso_frame; //use this to transform sensor frame to torso frame
    tf::TransformListener tf_listener; //start a transform listener

    //let's warm up the tf_listener, to make sure it get's all the transforms it needs to avoid crashing:
    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and torso...");
    while (tferr) {
        tferr = false;
        try {

            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tf_listener.lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good"); //tf-listener found a complete chain from sensor to world; ready to roll
    //convert the tf to an Eigen::Affine:
    Eigen::Affine3f A_sensor_wrt_torso;
    Eigen::Affine3d Affine_des_gripper;
    Eigen::Vector3d xvec_des,yvec_des,zvec_des,origin_des;
    geometry_msgs::PoseStamped rt_tool_pose;

    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_cloud_display", 1);
    //pcl::PointCloud<pcl::PointXYZ> & outputCloud
    pcl::PointCloud<pcl::PointXYZ> display_cloud; // instantiate a pointcloud object, which will be used for display in rviz
    sensor_msgs::PointCloud2 pcl2_display_cloud; //(new sensor_msgs::PointCloud2); //corresponding data type for ROS message


    A_sensor_wrt_torso = cwru_pcl_utils.transformTFToEigen(tf_sensor_frame_to_torso_frame);
    // we don't need to have it returned; cwru_pcl_utils can own it as a member var
    cwru_pcl_utils.transform_kinect_cloud(A_sensor_wrt_torso);
    //save this transformed data to disk:
    cwru_pcl_utils.save_transformed_kinect_snapshot();
    Eigen::Vector3f plane_normal, major_axis, centroid;
    Eigen::Matrix3d Rmat;
    int rtn_val;
    double plane_dist;
    double incremental_dist_x;
    double incremental_dist_y;
    Eigen::Vector3d right_up_cnr,
                            right_dn_cnr,
                            left_up_cnr,
                            left_dn_cnr;
    std::vector<Eigen::Vector3d> points_on_path_vec;
    int npts_on_path;
    //send a command to plan a joint-space move to pre-defined pose:
    rtn_val=arm_motion_commander.plan_move_to_pre_pose();
    
    //send command to execute planned motion
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    
    while (ros::ok())
    {
        if (cwru_pcl_utils.got_selected_points())
        {
            ROS_INFO("transforming selected points");
            cwru_pcl_utils.transform_selected_points_cloud(A_sensor_wrt_torso);

            //fit a plane to these selected points:
            cwru_pcl_utils.fit_xformed_selected_pts_to_plane(plane_normal, plane_dist);
            ROS_INFO_STREAM(" normal: " << plane_normal.transpose() << "; dist = " << plane_dist);
            cwru_pcl_utils.example_pcl_operation(); // offset the transformed, selected points and put result in gen-purpose object
            cwru_pcl_utils.get_gen_purpose_cloud(display_cloud);
            
            pcl::toROSMsg(display_cloud, pcl2_display_cloud); //convert datatype to compatible ROS message type for publication
            pcl2_display_cloud.header.stamp = ros::Time::now(); //update the time stamp, so rviz does not complain        
            pubCloud.publish(pcl2_display_cloud); //publish a point cloud that can be viewed in rviz (under topic pcl_cloud_display)
            ros::spinOnce();

            major_axis= cwru_pcl_utils.get_major_axis();
            centroid = cwru_pcl_utils.get_centroid();
            cwru_pcl_utils.reset_got_selected_points();   // reset the selected-points trigger
            //construct a goal affine pose:
            for (int i=0;i<3;i++) {
                origin_des[i] = centroid[i]; // convert to double precision
                zvec_des[i] = -plane_normal[i]; //want tool z pointing OPPOSITE surface normal
                xvec_des[i] = major_axis[i];
            }
            origin_des[2]+=0.02; //raise up 2cm
            yvec_des = zvec_des.cross(xvec_des); //construct consistent right-hand triad
            Rmat.col(0)= xvec_des;
            Rmat.col(1)= yvec_des;
            Rmat.col(2)= zvec_des;
            Affine_des_gripper.linear()=Rmat;
            Affine_des_gripper.translation()=origin_des;
            cout<<"des origin: "<<Affine_des_gripper.translation().transpose()<<endl;
            cout<<"orientation: "<<endl;
            cout<<Rmat<<endl;
            //convert des pose from Eigen::Affine to geometry_msgs::PoseStamped
            rt_tool_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
            //could fill out the header as well...
            
            // send move plan request:
            rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
            if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS)  { 
                //send command to execute planned motion
                rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
            }
            else {
                ROS_WARN("Cartesian path to desired pose not achievable");
            }
            cwru_pcl_utils.find_four_corners(right_up_cnr, right_dn_cnr, left_up_cnr, left_dn_cnr);
            ROS_INFO_STREAM(right_up_cnr.transpose() << right_dn_cnr.transpose()  << left_up_cnr.transpose()  << left_dn_cnr.transpose());
            
            arm_motion_commander.compute_path(right_up_cnr, right_dn_cnr, left_up_cnr, left_dn_cnr, points_on_path_vec);
            npts_on_path = points_on_path_vec.size();
            ROS_INFO("the number of points on path %d", npts_on_path);

            for (int i = 0; i < npts_on_path; ++i)
            {
                Affine_des_gripper.translation() = points_on_path_vec[i];
                rt_tool_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
                rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
                if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS)
                {
                //send command to execute planned motion
                    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
                }
                else
                {
                    ROS_WARN("Cartesian path to desired pose not achievable");
                }
            }
            ros::Duration(1).sleep(); // sleep for half a second
            rtn_val=arm_motion_commander.plan_move_to_pre_pose();
            //send command to execute planned motion
            rtn_val=arm_motion_commander.rt_arm_execute_planned_path();

        //     incremental_dist_x = 0.2;
        //     incremental_dist_y = 0.05;
        //     // from current pose, the gripper will move 10cm back and forth along its 
        //     for (int i = 0; i < 10; ++i)
        //     {
        //         if (origin_des[1] < origin_des[1] + 4 * incremental_dist_y)
        //         {
        //             origin_des[0] = origin_des[0] + incremental_dist_x;  // move 10cm
        //             Affine_des_gripper.translation()=origin_des;
        //             rt_tool_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
        //             rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
        //             if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS)
        //             {
        //                 //send command to execute planned motion
        //                 rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
        //             }
        //             origin_des[1] = origin_des[1] + incremental_dist_y;  // move 10cm
        //             Affine_des_gripper.translation()=origin_des;
        //             rt_tool_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
        //             rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
        //             if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS)
        //             {
        //                 //send command to execute planned motion
        //                 rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
        //             }
        //             origin_des[0] = origin_des[0] - incremental_dist_x; // move -10cm
        //             Affine_des_gripper.translation()=origin_des;
        //             rt_tool_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
        //             rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
        //             if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS)
        //             {
        //                 //send command to execute planned motion
        //                 rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
        //             }
        //             origin_des[1] = origin_des[1] + incremental_dist_y;  // move 10cm
        //             Affine_des_gripper.translation()=origin_des;
        //             rt_tool_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
        //             rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
        //             if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS)
        //             {
        //                 //send command to execute planned motion
        //                 rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
        //             }
        //             ROS_INFO("This is the NO. %d repeatation", ++i);
        //         }
        //     }
        //     ros::Duration(1).sleep(); // sleep for half a second
        //     rtn_val=arm_motion_commander.plan_move_to_pre_pose();
        //     //send command to execute planned motion
        //     rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
        }
        // pcl::toROSMsg(display_cloud, pcl2_display_cloud); //convert datatype to compatible ROS message type for publication
        // pcl2_display_cloud.header.stamp = ros::Time::now(); //update the time stamp, so rviz does not complain        
        // pubCloud.publish(pcl2_display_cloud); //publish a point cloud that can be viewed in rviz (under topic pcl_cloud_display)
        ros::Duration(0.5).sleep(); // sleep for half a second
        ros::spinOnce();
    }
 
    return 0;
}


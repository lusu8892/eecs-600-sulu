# pcl_object_finder
 A library to illustrate use of some point-cloud operations.
Example "main" program use of this library in pcl_object_finder_main.cpp.
Illustrates point-cloud saving to disk, transformation to alternative frame, and use of point-cloud selection.

## Example usage
`roslaunch cwru_baxter_sim baxter_world.launch`
`roslaunch cwru_baxter_sim kinect_xform.launch`
`rosrun pcl_object_finder pcl_object_finder_main`
`rosrun rviz rviz` (and set display to see kinect/depth/points)

Then select a patch of points in the rviz view.  Routine will display plane properties.
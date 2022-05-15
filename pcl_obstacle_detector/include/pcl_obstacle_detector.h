#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection3D.h>
#include <sensor_msgs/PointCloud2.h>
//PCL includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>


class Obstacle_Detector
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher obstacle_pub_;
    ros::Publisher pt_pub_;

public:
    Obstacle_Detector();
    void cloud_callback(const sensor_msgs::PointCloud2 &input);
};

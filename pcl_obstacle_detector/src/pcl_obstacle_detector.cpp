#include "pcl_obstacle_detector.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>


Obstacle_Detector::Obstacle_Detector(std::string name):
as_(nh_, name, false),
action_name_(name)
{
    as_.registerGoalCallback(boost::bind(&Obstacle_Detector::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&Obstacle_Detector::preemptCB, this));
    point_cloud_sub_ = nh_.subscribe("/xtion/depth_registered/points", 10, &Obstacle_Detector::cloud_callback, this);
    obstacle_pub_ = nh_.advertise<vision_msgs::Detection3DArray>("/pcl_obstacle_detector_node/detections", 10);
    pt_pub_= nh_.advertise<sensor_msgs::PointCloud2> ("/pcl_obstacle_detector_node/pcl_output", 1);
    as_.start();
}
Obstacle_Detector::~Obstacle_Detector()
{
  as_.shutdown();
}
void Obstacle_Detector::cloud_callback(const sensor_msgs::PointCloud2 &input)
{
    if (!as_.isActive())
      return;
    if (as_.isPreemptRequested())
      return;
    if (goal_!=1)
      return;
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);
    pcl::fromROSMsg(input, *cloud);
    pcl::PassThrough<pcl::PointXYZ> pass;//设置滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass1;
    pcl::PassThrough<pcl::PointXYZ> pass2;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName ("x");//x axis pointing right
    pass.setFilterLimits (-10.0, 10.0);
    //pass.setFilterLimitsNegative (false);//设置保留范围内的还是过滤掉范围内的:算法内部默认false，即保留范围内的，滤掉范围外的；若设为true，则保留范围外的，滤掉范围内的；
    //pass.setNegative (true);//作用同setFilterLimitsNegative
    pass.filter (*cloud);
    pass1.setInputCloud(cloud);
    pass1.setFilterFieldName ("z");//z axis pointing outward
    pass1.setFilterLimits (0.1, 25.0);
    pass1.filter (*cloud);
    pass2.setInputCloud(cloud);
    pass2.setFilterFieldName ("y");//y axis pointing downward
    pass2.setFilterLimits (-1.1, 0.9);
    pass2.filter (*cloud);

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);			//设置滤波对象的需要的点云
    sor.setLeafSize (0.01f, 0.01f, 0.01f);  //设置滤波创建的体素大小为1cm
    sor.filter (*cloud);
    // std::cout<<"size of pcl="<<sizeof(*cloud)<<std::endl;
    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // // Create the segmentation object
    // pcl::SACSegmentation<pcl::PointXYZ> seg;
    // seg.setOptimizeCoefficients(true);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setDistanceThreshold(0.3);

    // seg.setInputCloud(cloud);
    // seg.segment(*inliers, *coefficients);

    // if (inliers->indices.size() == 0)
    // {
    //     PCL_ERROR("Could not estimate ground plane.");
    //     std::exit(1);
    // }

    // // Filtered the ground points
    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    // extract.setInputCloud(cloud);
    // extract.setIndices(inliers);
    // extract.setNegative(true);
    // extract.filter(*cloud);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.075);
    ec.setMinClusterSize(7);
    ec.setMaxClusterSize(80000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int i = 0;
    vision_msgs::Detection3DArray barrel_msg;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx : it->indices)
            cloud_cluster->push_back((*cloud)[idx]); //*


        Eigen::Vector4f centroid;
        Eigen::Vector4f min;
        Eigen::Vector4f max;

        pcl::compute3DCentroid(*cloud_cluster, centroid);
        pcl::getMinMax3D(*cloud_cluster, min, max);
    
        // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
        // std::cout << "position X: " << centroid[0] << std::endl;
        // std::cout << "position Y: " << centroid[1] << std::endl;
        // std::cout << "position Z: " << centroid[2] << std::endl;

        vision_msgs::Detection3D box_msg;

        box_msg.bbox.center.position.x = centroid[0];
        box_msg.bbox.center.position.y = centroid[1];
        box_msg.bbox.center.position.z = centroid[2];
        box_msg.bbox.size.x = (max[0] - min[0]);
        box_msg.bbox.size.y = (max[1] - min[1]);
        box_msg.bbox.size.z = (max[2] - min[2]);
        if (box_msg.bbox.size.y > 2.0 || box_msg.bbox.size.x > 1.2 || box_msg.bbox.size.z > 1.2 || box_msg.bbox.size.x < 0.25 || box_msg.bbox.size.z < 0.25) {
            continue;
        }
        box_msg.header.frame_id = input.header.frame_id;
        box_msg.header.stamp = input.header.stamp;

        barrel_msg.detections.push_back(box_msg);
    }

    obstacle_pub_.publish(barrel_msg);
    pt_pub_.publish(output);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pcl_obstacle_detector_node");

    Obstacle_Detector obstacle_detector("person_detector");

    // Spin
    ros::spin();
}
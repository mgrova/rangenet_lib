#include "SegmentationNode.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

namespace seg
{
SegmentationNode::SegmentationNode(const ros::NodeHandle &nh) : _nh(nh)
{
    std::string cloud_sub, segmentation_pub, model_path, backend;
    int verbose;
    ros::param::param<std::string>("cloud_sub", cloud_sub, "/velodyne_points");
    ros::param::param<std::string>("segmentation_pub", segmentation_pub, "segmentation");
    ros::param::param<std::string>("model_path", model_path, "/home/user/shared_folder/darknet53/");
    ros::param::param<std::string>("backend", backend, "tensorrt");
    ros::param::param<int>("verbose", verbose, 1);

    _cloud_sub      = _nh.subscribe(cloud_sub, 10, &SegmentationNode::segmentationCb, this);
    _detections_pub = _nh.advertise<sensor_msgs::PointCloud2>(segmentation_pub, 1);

    /// \note. Ensure that path finish with "/"
    std::string value {"/"};
    if (!std::equal(value.rbegin(), value.rend(), model_path.rbegin()))
        model_path += value;

    try
    {
        _net = std::move(rangenet::segmentation::make_net(model_path, backend));
        const auto verbose_bool = verbose == 1 ? true : false;
        _net->verbosity(verbose_bool);

        std::cout << "Successfully loaded model from: "
                  << model_path << " with backend: " << backend << std::endl;
    } 
    catch (...) 
    {
        throw;
    }
}

void SegmentationNode::segmentationCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);

    std::vector<float> cloudVector;
    for (const auto& point : cloud->points) 
    {
        cloudVector.push_back(point.x); cloudVector.push_back(point.y);
        cloudVector.push_back(point.z); cloudVector.push_back(point.intensity);
    }

    std::vector<std::vector<float>> semantic_scan = _net->infer(cloudVector, cloud->points.size());

    std::vector<cv::Vec3f> points = _net->getPoints(cloudVector, cloud->points.size());
    std::vector<cv::Vec3b> color_mask = _net->getLabels(semantic_scan, cloud->points.size());

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr detections_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (unsigned int i=0; i<points.size(); i++) 
    {
		// get the RGB color value for the point
		cv::Vec3b rgbv(255,255,255);
		if (color_mask.size() > i) 
			rgbv = color_mask[i];

		pcl::PointXYZRGB pclp;
		// 3D coordinates
		pclp.x = points[i](0);
		pclp.y = points[i](1);
		pclp.z = points[i](2);
		
		// RGB color, needs to be represented as an integer
		uint32_t rgb = ((uint32_t)rgbv[2] << 16 | (uint32_t)rgbv[1] << 8 | (uint32_t)rgbv[0]);
		pclp.rgb = *reinterpret_cast<float*>(&rgb);
		
		detections_cloud->push_back(pclp);
	}
	
	detections_cloud->width = (uint32_t) detections_cloud->points.size(); // number of points
	detections_cloud->height = 1;								          // a list, one row of data
    
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*detections_cloud, ros_cloud);
    ros_cloud.header.frame_id = cloud_msg->header.frame_id;
    ros_cloud.header.stamp = ros::Time::now();

    _detections_pub.publish(ros_cloud);
}

 
}
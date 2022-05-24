#include "SegmentationNode.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

namespace seg
{
SegmentationNode::SegmentationNode(const ros::NodeHandle &nh) : _nh(nh)
{
    _cloudSub      = _nh.subscribe("topic_name", 10, &SegmentationNode::segmentationCb, this);
    _detectionsPub = _nh.advertise<sensor_msgs::PointCloud2>("segmentation", 1);
}

bool SegmentationNode::initializeModel(const bool &verbose, const std::string &path, const std::string &backend)
{
    try {

        _net = std::move(rangenet::segmentation::make_net(path, backend));
        _net->verbosity(verbose);

        return true;
    } 
    catch (const std::runtime_error &ex) 
    {
        std::cerr << ex.what() << std::endl;
        return false;
    }
}

void SegmentationNode::segmentationCb(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloudMsg, *cloud);
    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);

    std::vector<float> cloudVector;
    for (const auto& point : cloud->points) 
    {
        cloudVector.push_back(point.x); cloudVector.push_back(point.y);
        cloudVector.push_back(point.z); cloudVector.push_back(point.intensity);
    }

    std::vector<std::vector<float>> semantic_scan = net->infer(cloudVector, cloud->points.size());

    std::vector<cv::Vec3f> points = net->getPoints(cloudVector, cloud->points.size());
    std::vector<cv::Vec3b> color_mask = net->getLabels(semantic_scan, cloud->points.size());

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr detections_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (unsigned int i=0; i<points.size(); i++) 
    {
		// get the RGB color value for the point
		cv::Vec3b rgbv(255,255,255);
		if (color_mask.size() > i) 
        {
			rgbv = color_mask[i];
		}

		/// \todo. check for erroneous coordinates (NaN, Inf, etc.)
		
		pcl::PointXYZRGB pclp;
		// 3D coordinates
		pclp.x = points[i].x;
		pclp.y = points[i].y;
		pclp.z = points[i].z;
		
		// RGB color, needs to be represented as an integer
		uint32_t rgb = ((uint32_t)rgbv[2] << 16 | (uint32_t)rgbv[1] << 8 | (uint32_t)rgbv[0]);
		pclp.rgb = *reinterpret_cast<float*>(&rgb);
		
		detections_cloud->push_back(pclp);
	}
	
	detections_cloud->width = (uint32_t) detections_cloud->points.size(); // number of points
	detections_cloud->height = 1;								          // a list, one row of data
    
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*detections_cloud, ros_cloud);
    ros_cloud.header.frame_id = "map";
    ros_cloud.header.stamp = ros::Time::now();

    _detectionsPub.publish(ros_cloud);
}

 
}
#ifndef SEGMENTATION_NODE_H_
#define SEGMENTATION_NODE_H_

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <selector.hpp>

namespace seg
{

class SegmentationNode
{
public:
    explicit SegmentationNode(const ros::NodeHandle &);

private:
    void segmentationCb(const sensor_msgs::PointCloud2ConstPtr &);

private:    
    ros::NodeHandle _nh;

    ros::Subscriber _cloud_sub;
    ros::Publisher  _detections_pub;
    
    std::unique_ptr<rangenet::segmentation::Net> _net;
};
}


#endif
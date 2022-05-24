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
    bool initializeModel(const bool &, const std::string &, const std::string &backend = "tensorrt");
    
private:
    void segmentationCb(const sensor_msgs::PointCloud2ConstPtr &);

private:    
    ros::NodeHandle _nh;

    ros::Subscriber _cloudSub;
    ros::Publisher  _detectionsPub;
    
    std::unique_ptr<rangenet::segmentation::Net> _net;
};
}


#endif
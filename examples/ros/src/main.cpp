#include <iostream>
#include <memory>

#include "SegmentationNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sloam");
    ros::NodeHandle n("sloam");

    auto node = std::make_unique<seg::SegmentationNode>(n);
    
    ros::spin();

    return 0;
}
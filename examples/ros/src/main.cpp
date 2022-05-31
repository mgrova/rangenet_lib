#include <iostream>
#include <memory>

#include "SegmentationNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rangenet_node");
    ros::NodeHandle n("~");
    try
    {
        auto node = std::make_unique<seg::SegmentationNode>(n);
        ros::spin();
    }
    catch(const std::runtime_error &ex)
    {
        std::cerr << "Unable to load model: " << ex.what() << std::endl;
        return 0;
    }
}
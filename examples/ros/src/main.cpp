#include <iostream>
#include <memory>

#include "SegmentationNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rangenet_node");
    ros::NodeHandle n("rangenet_node");

    const auto path_folder {"/home/user/shared_folder/darknet53"};
    const auto backend {"tensorrt"};
    auto node = std::make_unique<seg::SegmentationNode>(n);
    if (!node->initializeModel(true, path_folder, backend))
    {
        std::cout << "Unable to load model. Going out ..." << std::endl;
        return 0;
    }

    ros::spin();

    return 0;
}
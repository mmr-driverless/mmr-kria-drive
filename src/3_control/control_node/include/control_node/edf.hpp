#ifndef CONTROLNODE_EDF_HPP
#define CONTROLNODE_EDF_HPP

#ifdef USE_EDF
#include <mmr_edf/mmr_edf.hpp>
#endif

namespace control_node {

#ifdef USE_EDF
using NodeBase = EDFNode;
#else
using NodeBase = rclcpp::Node;
#endif

}

#endif
#ifndef LOCAL_PLANNER_NODE_H
#define LOCAL_PLANNER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include "rcutils/time.h"
#include "rmw/macros.h"
#include "rmw/visibility_control.h"
#include "rmw/types.h"
#include "acceleration.h"
#include "autocross.h"
#include "skidpad.h"

#ifdef USE_EDF
#include <mmr_edf/mmr_edf.hpp>
using NodeBase = EDFNode;
#else
using NodeBase = rclcpp::Node;
#endif

using namespace std::chrono_literals;

class LocalPlannerNode : public NodeBase
{
	public:
		LocalPlannerNode();
		void loadParameters();
		void initialization();

	private:
		rclcpp::TimerBase::SharedPtr timer;

		rclcpp::Publisher<mmr_base::msg::MarkerArray>::SharedPtr  bordersPub;
		rclcpp::Publisher<mmr_base::msg::Marker>::SharedPtr  centerLinePub;
		rclcpp::Publisher<mmr_base::msg::MarkerArray>::SharedPtr  bordersCompletedPub;
		rclcpp::Publisher<mmr_base::msg::Marker>::SharedPtr  centerLineCompletedPub;

		rclcpp::Subscription<mmr_base::msg::RaceStatus>::SharedPtr raceStatusSub;
    	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySub;
		rclcpp::Subscription<mmr_base::msg::Marker>::SharedPtr slamConesSub;

		std::shared_ptr<AccelerationPlanner> accelerationPlanner;
		std::shared_ptr<AutocrossPlanner> autocrossPlanner;
		std::shared_ptr<SkidpadPlanner> skidpadPlanner;

		std::string param_eventType;
		std::string param_topicBorders;
		std::string param_topicCenterLine;
		std::string param_topicBordersCompleted;
		std::string param_topicCenterLineCompleted;
		std::string param_topicRaceStatus;
		std::string param_topicOdometry;
		std::string param_topicSlamCones;
};

#endif //LOCAL_PLANNER_NODE_H
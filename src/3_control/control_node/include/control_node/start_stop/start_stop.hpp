#ifndef CONTROLNODE_CONTROL_START_STOP_START_STOP_HPP
#define CONTROLNODE_CONTROL_START_STOP_START_STOP_HPP

#include <control_node/control/control.hpp>
#include <control_node/estimation/ivehicle_state.hpp>
#include <control_node/parameters.hpp>
#include "mmr_base/configuration.hpp"
#include <rclcpp/rclcpp.hpp>


namespace control_node {
namespace start_stop {

    enum StartStopStates{
        FIRST_GEAR,
        STARTING,
        DRIVING,
        GEAR_DOWN,
        NEUTRAL,
    };

    class StartStop {
        StartStopStates m_state;
        int m_lapTarget;
        float m_brakeToStop;
        rclcpp::Time timeStartLaunch;
        float m_timeForLaunch;
        bool isLaunchStarted = false;

    public:
        StartStop(const Parameters& p) : m_state(STARTING), 
                                         m_lapTarget(p.get<int>("lapTarget")),
                                         m_timeForLaunch(p.get<float>("timeForLaunch")),
                                         m_brakeToStop(p.get<float>("brakeToStop")) {}

        void triggerFSM(const estimation::IVehicleState& vehicle_state, control::Control& control);
    };

}; // namespace start_stop
}; // namespace control_node

#endif // !CONTROLNODE_CONTROL_START_STOP_START_STOP_HPP
#include <control_node/start_stop/start_stop.hpp>

namespace control_node {
namespace start_stop {

    void StartStop::triggerFSM(const estimation::IVehicleState& vehicle_state, control::Control& control){
        switch (m_state)
        {
        case FIRST_GEAR:
        if(vehicle_state.as_state() == AS::STATE::DRIVING){
            control.brake = m_brakeToStop;
            control.clutch = 1.0;
            control.gear = 1.0;

            m_state = STARTING;
        }
        break;
        case STARTING:
            if(!isLaunchStarted){
                timeStartLaunch = rclcpp::Time();
                isLaunchStarted = true;
            }
            if(rclcpp::Time().nanoseconds() < timeStartLaunch.nanoseconds() + m_timeForLaunch * 1e9){
                control.launch = true;
                control.throttle = 0.1;
                control.brake = 0.0;
            }
            else
                m_state = DRIVING;
            
            break;
        
        case DRIVING:
            if(vehicle_state.lap() >= m_lapTarget || vehicle_state.as_state() == AS::STATE::FINISHED)
                m_state = GEAR_DOWN;
            break;
        
        case GEAR_DOWN:
            if(control.gear <= 1){
                control.brake = m_brakeToStop;
                m_state = NEUTRAL;
            }
            else
                control.gear--;
            break;

        case NEUTRAL:
            control.brake = m_brakeToStop;
            control.clutch = 1.0;
            break;

        default:
            break;
        }
    }
}; // namespace start_stop
}; // namespace control_node
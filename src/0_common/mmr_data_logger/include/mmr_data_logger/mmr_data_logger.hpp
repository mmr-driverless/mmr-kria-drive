#pragma once

#include <mmr_edf/mmr_edf.hpp>

#include <string.h>

class MMR_Data_Logger : public EDFNode
{
    private:
    std::string steerTopic, brakeTopic, clutchTopic, statusActuatorTopic, ecuStatusTopic, xsenseTopic, asTopic, missionTopic, lapCounterTopic, conesActualTopic, conesAllTopic, controlTopic;

    /* DV driving dynamics 1 */ 

    uint8_t speedTarget, speedActual, brakeActual, brakeTarget;
    int8_t steeringAgleActual,steeringAngleTarget;

    /* DV driving dynamics 2 */

    int16_t accelerationLongitudinal, accelerationLateral, yawRate;

    /* DV system status */

    int8_t asStatus, ebsState, missionSelected, serviceBrakeState;
    bool steeringState;
    int8_t lapCounter, conesCountActual, conesCountAll;

    



    public:

};
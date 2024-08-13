#pragma once

#include <canopen_bridge/driver/lib/motor_maxon.hpp>

class MaxonBrake : public MaxonMotor
{
    private:

        const int m_nModeOfOp = MOTOR::CST;
        int m_nMaxTorque, m_nReturnPedalTorque;

        void initBrake() {
            /* set modes of operation */
            this->download<uint8_t>(0x6060, 0x00, this->m_nModeOfOp);

            /* enable device */
            this->init();
        }

    public:

        MaxonBrake(int nSocket, int nNodeId, int nTimeOutMsg, int nMaxTorque, int nReturnPedalTorque)
            : MaxonMotor(nSocket, nNodeId, m_nModeOfOp, nTimeOutMsg) 
        {
            this->m_nMaxTorque = nMaxTorque;
            this->m_nReturnPedalTorque = nReturnPedalTorque;

            for (int i = 0; i < 10; i++)
                this->initBrake();
        }

        ~MaxonBrake() { this->disable(); };

        void writeTargetTorque(double fTargetTorque) {

            fTargetTorque = (fTargetTorque * 1000.0 * 1000.0) / 928.0;
            int nTorque = std::clamp<double>(fTargetTorque, 0, this->m_nMaxTorque) * -1.0;
            int16_t unTorque = std::clamp<int>(nTorque, std::numeric_limits<int16_t>::min(), std::numeric_limits<int16_t>::max());

            /* set velocity */
            this->download<int16_t>(0x60B2, 0x00, unTorque);
        }

        void returnToZero() {
            uint16_t nButton = this->upload<uint16_t>(0x3141, 0x01);

            if (nButton)
                this->writeTargetTorque(0);
            else
                this->writeTargetTorque(this->m_nReturnPedalTorque);
        }
};
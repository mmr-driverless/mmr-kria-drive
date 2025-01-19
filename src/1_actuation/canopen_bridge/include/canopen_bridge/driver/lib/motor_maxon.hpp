#pragma once

#include <canopen_bridge/driver/lib/canopen.hpp>
#include "mmr_base/configuration.hpp"

#include <ctime>
#include <cmath>
#include <unistd.h>
#include <algorithm>
#include <iostream>
#include <numeric>

class MaxonMotor
{
    private:

        const int FAULT_MASK = 128;
        int socket, node_id, timeout_msg_count, mode_of_op;
        int send_msg_on_canbus(CANOpen::canopen_frame cof, CANOpen::canopen_frame* rcv);

    protected:

        MaxonMotor(int socket, int node_id, int mode_of_op, int timeout_msg_count=5);
        ~MaxonMotor() {};

        template<typename T>
        MOTOR::ERROR_IDX download(uint16_t index, uint8_t subindex, T value) {
            CANOpen::canopen_frame tx, rx;
            if ((CANOpen::instanceof<double>(value)) || (CANOpen::instanceof<long long int>(value)))
                return MOTOR::ERROR_IDX::FAILED_MSG_TOO_LONG;
            
            tx.id = MOTOR::REQUEST_SDO + this->node_id;
            tx.header = CANOpen::create_sdo_download_header(sizeof(T));
            tx.index = index;
            tx.subindex = subindex;

            memset(tx.data, 0, CANOpen::max_data_len * sizeof(uint8_t));
            memcpy(tx.data, &value, sizeof(T));

            if (this->send_msg_on_canbus(tx, &rx) < 0) return MOTOR::ERROR_IDX::FAILED_CAN;
            if ((rx.header & this->FAULT_MASK) == this->FAULT_MASK) return MOTOR::ERROR_IDX::FAILED_MAXON_IDX;

            // TODO: gestire nel caso in cui (rx->header >> (header_size - 1)) != 0
            //       è un errore -> cambiare stato del motore dello sterzo

            return MOTOR::ERROR_IDX::OK_IDX;
        }

        void init();
        void disable();

        void toggle_new_pos(MOTOR::IDX_TOGGLE_NEW_POS nType);

        /* needs to restore fault error */
        void clearFault() { this->download<uint16_t>(0x6040, 0x00, 0x0080); }

    public:

        template<typename T>
        T upload(uint16_t index, uint8_t subindex) {
            T res;
            CANOpen::canopen_frame tx, rx;

            tx.id = MOTOR::REQUEST_SDO + this->node_id;
            tx.header = CANOpen::sdo_upload_header;
            tx.index = index;
            tx.subindex = subindex;

            if (this->send_msg_on_canbus(tx, &rx) < 0) return -1;

            // TODO: gestire nel caso in cui (rx->header >> (header_size - 1)) != 0
            //       è un errore -> cambiare stato del motore dello sterzo

            if (sizeof(T) != (CANOpen::max_data_len - (rx.header >> 2) & 3)) return -1;
            memcpy(&res, rx.data, sizeof(T));
            return res;
        }
};
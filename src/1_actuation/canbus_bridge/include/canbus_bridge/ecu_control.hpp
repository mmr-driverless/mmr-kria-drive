#include <thread>
#include <mutex>
#include <functional>
#include <pthread.h>
#include <cstring>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/syscall.h>
#include <unistd.h>


class EcuControl 
{
    struct sched_attr {
        uint32_t size;
        uint32_t sched_policy;
        uint64_t sched_flags;
        int32_t sched_nice;
        uint32_t sched_priority;
        uint64_t sched_runtime;
        uint64_t sched_deadline;
        uint64_t sched_period;
    };

    private:

        int m_nSocket, m_nCtrUp = 0, m_nCtrDown = 0;
        int m_nCtrLimitUp, m_nCtrLimitDown, m_nDelay;
        std::mutex *m_mutex;
        struct can_frame frame;

        std::atomic_bool m_bRunning = false;

        std::jthread m_tExec;

        void threadCallback() {

            sched_attr attr = {
                .size = sizeof(attr),
                .sched_policy = SCHED_DEADLINE,
                .sched_runtime = (uint64_t)(static_cast<float>(this->m_nDelay) / 2.0) * (uint64_t)1000 * (uint64_t)1000,
                .sched_deadline = this->m_nDelay * (uint64_t)1000 * (uint64_t)1000,
                .sched_period = this->m_nDelay * (uint64_t)1000 * (uint64_t)1000,
            };

            if (syscall(SYS_sched_setattr, gettid(), &attr, 0) != 0) return;
            
            while (1) {

                this->m_bRunning.store(true);
                
                if (this->m_nCtrUp < this->m_nCtrLimitUp) {
                    std::unique_lock<std::mutex> lock(*this->m_mutex);
                    write(this->m_nSocket, &this->frame, sizeof(struct can_frame));
                    this->m_nCtrUp ++;
                }
                else if (this->m_nCtrDown < this->m_nCtrLimitDown) {
                    struct can_frame msg = {
                        .can_id = this->frame.can_id,
                        .can_dlc = this->frame.can_dlc,
                    };
                    memset(msg.data, 0x00, CAN_MAX_DLEN);
                    std::unique_lock<std::mutex> lock(*this->m_mutex);
                    write(this->m_nSocket, &msg, sizeof(struct can_frame));
                    this->m_nCtrDown ++;
                }
                else {
                    this->m_bRunning.store(false);
                    this->m_nCtrDown = this->m_nCtrUp = 0;
                    return;
                }

                sched_yield();
            }
        }

    public:

        /**
         * @param nDelay: set in milliseconds
         */
        EcuControl(int nSocket, std::mutex *mutex, int nTimeUp, int nTimeDown, int nDelay, struct can_frame ecu_frame)
        {
            this->m_nSocket = nSocket;
            this->m_mutex = mutex;
            this->m_nCtrLimitUp = nTimeUp;
            this->m_nCtrLimitDown = nTimeDown;
            this->frame = ecu_frame;
            this->m_nDelay = nDelay;            
        }

        int startCtr() {
            
            if (this->m_bRunning)
                return -1;

            this->m_tExec = std::jthread(std::bind(&EcuControl::threadCallback, this));

            return 0;
        }

        bool isRunning() { return this->m_bRunning; }
};
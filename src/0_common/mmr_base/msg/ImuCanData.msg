# IMU ROS Message dumped from CAN Bus 


################################################################
#            GROUP INFORMATION AND TIMESTAMP MESSAGES
################################################################

# error_code: Indicates error conditions. Possible values:
# - 0x00 [CEI_NoError]              ==> IMU is operating correctly
# - 0x01 [CEI_OutputBufferOverflow] ==> Output Buffer is full, at least one message was dropped
uint8 error_code

# sample_time: Timestamp expressed in 10 kHz (100 μs) clock ticks
uint32 sample_time

# group_counter: CAN Frame counter
uint16 group_counter

# utc_time: Time in UTC format [needs GPS Fix]
UTCTime utc_time

#################################################################
#                        STATUS MESSAGES
#################################################################

# status_word: Refer to https://xsens.com/xsens-mti-documentation for an explanation of the contents of this message
uint32 status_word

#################################################################
#                    ORIENTATION MESSAGES
#################################################################

# quaternion: IMU Orientation in Quaternion format
Quaternion quaternion # [N/A]

# euler_angles: IMU Orientation in Euler format
EulerAngles euler_angles # [deg]

#################################################################
#                    INERTIAL DATA MESSAGES
#################################################################

# rate_of_turn: IMU Rate of turn
RateOfTurn rate_of_turn

# acceleration: IMU Acceleration
Acceleration acceleration # [m / s^2]

#################################################################
#                        PRESSURE MESSAGES
#################################################################

# pressure: Exterior pressure
uint32 pressure # [Pa]

#################################################################
#                POSITION AND VELOCITY MESSAGES
#################################################################

# gnss_position: IMU Position in lat, lon
Gnss gnss_position # [deg]

# velocities: IMU x,y,z velocities
Velocity velocities # [m / s]

# gnss_status: Status of GNSS data
GnssStatus gnss_status
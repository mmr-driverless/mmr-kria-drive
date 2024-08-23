X sense

DV driving dynamics

Acceleration longitudinal X

Acceleration lateral Y

Yaw rate

EQ

Speed_actual

Steering_angle_actual    

Brake_hydr_actual

nodo controllo

Speed_target

Steering_angle_target

Brake_hydr_target

AS Status

AS_state_off 1
AS_state_ready 2
AS_state_driving bit 0-2 3
AS_state_emergency_brake 4
AS_state_finish

guardo p e bs da EQ,

- se <0 [unaviable]
  
- se >0 [armed]
  
- se >0 e p_brake > 0 [activated]                                        
  

EBS_state_unavailable 
EBS_state_armed
EBS_state_activated

Mission Selected

AMI_state_acceleration 
AMI_state_skidpad 
AMI_state_trackdrive
AMI_state_braketest 
AMI_state_inspection 
AMI_state_autocross

Da actuator status

Steering_state

Da actuator status leggiamo l'enable del freno e se enable allora [enable]

Service_brake_state_disengaged
Service_brake_state_engaged
Service_brake_state_available

Race/planning/race_status

Lap_counter

perception/cones

Cones_count_actual

slam/cones_positions

Cones_count_all

Per arrivare alla size devo dimensionare l'array che ci arriva
#include <Eigen/Dense>

using PointT = Eigen::Vector2d;

static inline double normalizeAngle(double angle){
  while(angle > M_PI) angle -= (2 * M_PI);
  while(angle < -M_PI) angle += (2 * M_PI);
  return angle;
}

#define carWheelbase 1.541          //m
#define carFrontWheelToFromCG 0.815 //m //FROM CAR CG TO FRONT WHEEL
#define carRearWheelToCG 0.75
#define carMaxSteerAngle 0.3979351

static inline double calculateSteeringTarget(PointT target, PointT car_position, double car_yaw, double lookforward, double steer_gain, double max_steer = carMaxSteerAngle, double com_dist_to_rear = carRearWheelToCG, double wheelbase = carWheelbase)
{
  PointT car_rear = car_position - Eigen::Vector2d(std::cos(car_yaw), std::sin(car_yaw)) * com_dist_to_rear;

  //Calculate delta between target direction and car Rotation
  double SteerTarget = normalizeAngle(atan2(target.y() - car_rear.y(), target.x() - car_rear.x()) - car_yaw);

  //Calculate steer target rotation
  double wheelRotation = atan2(2 * wheelbase * std::sin(SteerTarget) / (lookforward * steer_gain), 1);

  //Cut off with respect to real steer car capability
  return std::clamp(wheelRotation, -max_steer, max_steer);
}
#ifndef MOTOR_PUBLISHER_HPP
#define MOTOR_PUBLISHER_HPP

#include <cstdint>
#include <ros/node_handle.h>
#include <ros/ros.h>

class MotorPublisher {

public:
  MotorPublisher(uint32_t id, ros::NodeHandle& nh);
  MotorPublisher(const MotorPublisher& oth);
  ~MotorPublisher();

  uint32_t getId() const;
  void publishVelocity(float speed);

private:
  uint32_t id;
  ros::Publisher velocityPublisher;

};

#endif // MOTOR_PUBLISHER_HPP

#include "diana_powertrain/motor_publisher.hpp"

#include <std_msgs/Float32.h>

#include <team_diana_lib/strings/strings.h>

MotorPublisher::MotorPublisher(uint32_t id, ros::NodeHandle& nh) :
id(id)
{
  velocityPublisher = nh.advertise<std_msgs::Float32>(Td::toString("wheel_", id, "_speed"), 100);
}

MotorPublisher::~MotorPublisher()
{
  velocityPublisher.shutdown();
}

uint32_t MotorPublisher::getId() const
{
  return id;
}

void MotorPublisher::publishVelocity(float speed)
{
  std_msgs::Float32 msg;
  msg.data = speed;
  velocityPublisher.publish(msg);
}



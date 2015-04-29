#ifndef DIANAPOWERTRAINNODE_HPP
#define DIANAPOWERTRAINNODE_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "diana_powertrain/powertrain_manager.hpp"
#include "diana_powertrain/pci7841_card.hpp"
#include "diana_powertrain/EnableMotors.h"

// = n.subscribe("chatter", 1000, chatterCallback);
class DianaPowertrainNode {

public:
  DianaPowertrainNode(int argc, char** argv);
  ~DianaPowertrainNode();

  void run();

private:
  void setVelocityCallback(const geometry_msgs::Twist& msg);
  bool setEnableMotorsCallback(diana_powertrain::EnableMotors::Request& req, diana_powertrain::EnableMotors::Response& res);

private:
   ros::NodeHandle n;
   Pci7841Card card;
   PowertrainManager<Pci7841Card> manager;
   ros::Subscriber velocitySubscriber;
   ros::Publisher velocityPublisher;
   ros::ServiceServer enableMotorsService;
};

#endif // DIANAPOWERTRAINNODE_HPP

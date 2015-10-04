#ifndef DIANAPOWERTRAINNODE_HPP
#define DIANAPOWERTRAINNODE_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "diana_powertrain/powertrain_manager.hpp"
#include "diana_powertrain/pci7841_card.hpp"
#include "diana_powertrain/EnableMotors.h"
#include "diana_powertrain/GetStatusWord.h"
#include "diana_powertrain/SetControlWord.h"
#include "diana_powertrain/GetOperationMode.h"
#include "diana_powertrain/SetOperationMode.h"
#include "diana_powertrain/motor_publisher.hpp"

#include <thread>
#include <memory>

// = n.subscribe("chatter", 1000, chatterCallback);
class DianaPowertrainNode {

public:
  DianaPowertrainNode(int argc, char** argv);
  ~DianaPowertrainNode();

  void run();

  void setMsgAndServicesEnabled(bool enabled);

private:
  void setVelocityCallback(const geometry_msgs::Twist& msg);
  bool setEnableMotorsCallback(diana_powertrain::EnableMotors::Request& req, diana_powertrain::EnableMotors::Response& res);
  bool getStatusWordCallback(diana_powertrain::GetStatusWord::Request& req, diana_powertrain::GetStatusWord::Response& res);
  bool setControlWordCallback(diana_powertrain::SetControlWord::Request& req, diana_powertrain::SetControlWord::Response& res);
  bool getOperationModeCallback(diana_powertrain::GetOperationMode::Request& req, diana_powertrain::GetOperationMode::Response& res);
  bool setOperationModeCallback(diana_powertrain::SetOperationMode::Request& req, diana_powertrain::SetOperationMode::Response& res);
  void publishUpdate();


private:
   ros::NodeHandle n;
   Pci7841Card card;
   PowertrainManager manager;
   ros::Subscriber velocitySubscriber;
   ros::Publisher velocityPublisher;
   ros::ServiceServer enableMotorsService;
   ros::ServiceServer getStatusWordService;
   ros::ServiceServer setStatusWordService;
   ros::ServiceServer getOperationModeService;
   ros::ServiceServer setOperationModeService;
   std::unique_ptr<std::thread> publishUpdateThread;
   std::vector<MotorPublisher> motorPublishers;

};

#endif // DIANAPOWERTRAINNODE_HPP

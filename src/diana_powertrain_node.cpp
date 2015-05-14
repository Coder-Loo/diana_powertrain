#include <iostream>
#include <string>

#include <geometry_msgs/Twist.h>

#include <team_diana_lib/logging/logging.h>
#include <team_diana_lib/strings/strings.h>

#include "diana_powertrain/diana_powertrain_node.hpp"

#include "diana_powertrain/powertrain_manager.hpp"
#include "diana_powertrain/pci7841_card.hpp"

INITIALIZE_EASYLOGGINGPP

using namespace hlcanopen;
using namespace Td;

DianaPowertrainNode::DianaPowertrainNode(int argc, char** argv) :
  card(0, 0),
  manager(card)
{
  ros::NodeHandle n;

  // Create publisher
  velocityPublisher = n.advertise<geometry_msgs::Twist>("velocity", 1000);
}

DianaPowertrainNode::~DianaPowertrainNode() {

}

void DianaPowertrainNode::setMsgAndServicesEnabled(bool enabled)
{
  velocitySubscriber.shutdown();
  enableMotorsService.shutdown();
  getStatusWordService.shutdown();

  if(enabled) {
    // TODO: queue size = 1 only for testing, use 1000 after testing.
    velocitySubscriber = n.subscribe("set_velocity", 1, &DianaPowertrainNode::setVelocityCallback, this);
    enableMotorsService = n.advertiseService("enable_motors", &DianaPowertrainNode::setEnableMotorsCallback, this);
    getStatusWordService = n.advertiseService("get_status_word", &DianaPowertrainNode::getStatusWordCallback, this);
  }
}


void DianaPowertrainNode::setVelocityCallback(const geometry_msgs::Twist& msg) {
  ros_info(toString("Setting velocity: ", msg.linear.x, " - ", msg.angular.z));

  if(!manager.set_velocity(msg.linear.x, msg.angular.z)) {
    ros_warn("error while setting velocity.");
  }
}

bool DianaPowertrainNode::setEnableMotorsCallback(diana_powertrain::EnableMotors::Request& req,
                                                  diana_powertrain::EnableMotors::Response& res) {
  manager.set_motors_enabled(req.enable);
  res.ok = true;
  return true;
}

bool DianaPowertrainNode::getStatusWordCallback(diana_powertrain::GetStatusWord::Request& req,
                                                diana_powertrain::GetStatusWord::Response& res)
{
  // TODO: create response response, make it an array of 4 motor status value dict.
  manager.printMotorsStatusWord();
  return true;
}

bool DianaPowertrainNode::setControlWordCallback(diana_powertrain::SetControlWord::Request& req,
                                                 diana_powertrain::SetControlWord::Response& res)
{
  ControlWordCommand command;
  if(!getControlWordCommandFromString(req.command, command)) {
    ros_error("Unknown command word: " + req.command);
    res.ok = false;
    return false;
  }
  manager.setControlWord(command);
  return true;
}

bool DianaPowertrainNode::getOperationModeCallback(diana_powertrain::GetOperationMode::Request& req,
                                                   diana_powertrain::GetOperationMode::Response& res)
{
  // Improve response TODO:
  manager.printMotorsOperationMode();
  return true;
}

bool DianaPowertrainNode::setOperationModeCallback(diana_powertrain::SetOperationMode::Request& req, diana_powertrain::SetOperationMode::Response& res)
{
  ModeOfOperation mode;
  if(!getModeOfOperationFromString(req.operation_mode, mode)) {
    ros_error("Unknown operation mode: " + req.operation_mode);
    return false;
  }
  manager.setMotorsOperationMode(mode);
}

void DianaPowertrainNode::run() {
  ros_info("starting powertrain manager");
  manager.initiate_clients();

  manager.reset_motors();

  std::future<bool> ok = manager.set_motors_enabled(true);
//   ok.get();

  setMsgAndServicesEnabled(true);

  ros_info("spin");
  ros::spin();

  setMsgAndServicesEnabled(false);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "powertrain_node");
  DianaPowertrainNode node(argc, argv);

  node.run();
}

#include <iostream>
#include <string>

#include <geometry_msgs/Twist.h>

#include <team_diana_lib/logging/logging.h>
#include <team_diana_lib/strings/strings.h>
#include <team_diana_lib/strings/iterables.h>

#include "diana_powertrain/diana_powertrain_node.hpp"

#include "diana_powertrain/powertrain_manager.hpp"
#include "diana_powertrain/pci7841_card.hpp"

#include <cassert>

INITIALIZE_EASYLOGGINGPP

using namespace hlcanopen;
using namespace Td;

DianaPowertrainNode::DianaPowertrainNode(int argc, char** argv) :
  card(0, 0),
  manager(card)
{

  // Create publisher
  velocityPublisher = n.advertise<geometry_msgs::Twist>("velocity", 1000);
}

DianaPowertrainNode::~DianaPowertrainNode() {
  ros_info("Stopping publish update thread");
  publishUpdateThread->join();
  ros_info("powertrain node ended");
}

void DianaPowertrainNode::setMsgAndServicesEnabled(bool enabled)
{
  velocitySubscriber.shutdown();
  enableMotorsService.shutdown();
  getStatusWordService.shutdown();
  setOperationModeService.shutdown();
  getOperationModeService.shutdown();

  if(enabled) {
    // TODO: queue size = 1 only for testing, use 1000 after testing.
    velocitySubscriber = n.subscribe("set_velocity", 1, &DianaPowertrainNode::setVelocityCallback, this);
    enableMotorsService = n.advertiseService("enable_motors", &DianaPowertrainNode::setEnableMotorsCallback, this);
    setStatusWordService = n.advertiseService("set_control_word", &DianaPowertrainNode::setControlWordCallback, this);
    getStatusWordService = n.advertiseService("get_status_word", &DianaPowertrainNode::getStatusWordCallback, this);
    setOperationModeService = n.advertiseService("set_operation_mode", &DianaPowertrainNode::setOperationModeCallback, this);
    getOperationModeService = n.advertiseService("get_operation_mode", &DianaPowertrainNode::getOperationModeCallback, this);
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
  return true;
}

void DianaPowertrainNode::publishUpdate()
{
  std::vector<Motor<Pci7841Card>>& motors = manager.getMotors();
  for(Motor<Pci7841Card> motor : motors) {
    float vel = motor.getVelocity().get().value;
    auto it = std::find_if(motorPublishers.begin(), motorPublishers.end(), [&](const MotorPublisher& p) {
      return p.getId() == motor.getId();
    });
    assert(it != motorPublishers.end());
    MotorPublisher& publisher = *it;
    publisher.publishVelocity(vel);
  }
}

void DianaPowertrainNode::run() {
  ros_info("starting powertrain manager");


  std::vector<int> motorIds;
  if(!n.hasParam("motor_ids")) {
    motorIds.push_back(11);
    motorIds.push_back(12);
    motorIds.push_back(13);
    motorIds.push_back(14);
  } else {
    n.getParam("motor_ids", motorIds);
  }

  Td::ros_info("Starting motors: " + Td::iterableToString(motorIds));

  for(int motorId : motorIds) {
    motorPublishers.push_back(MotorPublisher(motorId, n));
  }

 // publishUpdateThread = std::unique_ptr<std::thread>(new std::thread(
 //   [&] () {
 //     while(ros::ok()) {
 //       publishUpdate();
 //       std::this_thread::sleep_for(std::chrono::milliseconds(500));
 //     }
 //   }
 // ));

  manager.initiate_clients(motorIds);

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

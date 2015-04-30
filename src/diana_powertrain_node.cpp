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

  if(enabled) {
    // TODO: queue size = 1 only for testing, use 1000 after testing.
    velocitySubscriber = n.subscribe("set_velocity", 1, &DianaPowertrainNode::setVelocityCallback, this);
    enableMotorsService = n.advertiseService("enable_motors", &DianaPowertrainNode::setEnableMotorsCallback, this);
  } else {
    velocitySubscriber.shutdown();
    enableMotorsService.shutdown();
  }
}


void DianaPowertrainNode::setVelocityCallback(const geometry_msgs::Twist& msg) {
  // TODO: replace this once timeout will be implemented inside hlcanopen
  std::future<bool> f = std::async(std::launch::async, [&, msg]() {
    ros_info(toString("Setting velocity: ", msg.linear.x, " - ", msg.angular.z));
    manager.set_velocity(msg.linear.x, msg.angular.z);
    return true;
  });
  if(f.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready) {
    ros_warn("Something went wrong while setting velocity");
  } else {
    ros_info("getting set velocity future value");
//     f.get();
  }
}

bool DianaPowertrainNode::setEnableMotorsCallback(diana_powertrain::EnableMotors::Request& req,
                                                  diana_powertrain::EnableMotors::Response& res) {
  manager.set_motors_enabled(req.enable);
  res.ok = true;
  return true;
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

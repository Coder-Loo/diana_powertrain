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
  velocitySubscriber = n.subscribe("set_velocity", 1000, &DianaPowertrainNode::setVelocityCallback, this);
  enableMotorsService = n.advertiseService("enable_motors", &DianaPowertrainNode::setEnableMotorsCallback, this);
}

DianaPowertrainNode::~DianaPowertrainNode() {

}

void DianaPowertrainNode::setVelocityCallback(const geometry_msgs::Twist& msg) {
  ros_info(toString("Setting velocity: ", msg.linear.x, " - ", msg.angular.z));
  manager.set_velocity(msg.linear.x, msg.angular.z);
}

bool DianaPowertrainNode::setEnableMotorsCallback(diana_powertrain::EnableMotors::Request& req,
                                                  diana_powertrain::EnableMotors::Response& res) {
  manager.set_motors_enabled(req.enable);
  res.ok = true;
}


void DianaPowertrainNode::run() {
  ros_info("starting powertrain manager");
  manager.initiate_clients();

  manager.run();

  manager.set_motors_enabled(true);

  ros_info("spin");
  ros::spin();
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "powertrain_node");
  DianaPowertrainNode node(argc, argv);

  node.run();
}

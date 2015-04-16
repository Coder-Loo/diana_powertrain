#include <iostream>
#include <string>

#include "diana_powertrain/powertrain_manager.hpp"
#include "diana_powertrain/pci7841_card.h"

INITIALIZE_EASYLOGGINGPP

using namespace hlcanopen;


int main(int argc, char** argv) {
  Pci7841Card card(0, 0);
  PowertrainManager<Pci7841Card> manager(card);

  manager.initiate_clients();
  manager.run();
}

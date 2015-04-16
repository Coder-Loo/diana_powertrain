#include "diana_powertrain/utils.hpp"

#include <unistd.h>

void mssleep(int ms)
{
  usleep(ms*1000);
}

#ifndef DIANA_POWERTRAIN_UTILS_HPP
#define DIANA_POWERTRAIN_UTILS_HPP

void mssleep(int ms);

template <typename T> T clamp(const T& v, const T& min, const T& max) {
  if( v > max) {
    return max;
  }
  if ( v < min) {
    return min;
  }
  return v;
}

#endif

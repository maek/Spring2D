#ifndef __SETTINGS_H__
#define __SETTINGS_H__

// Include all the external headers
#include <iostream>
#include <list>
#include <cmath>

// Double precision flag
#define S2_DOUBLE_PRECISION 0


// Create the Spring2D namespace & define a shorter synonym for it
namespace Spring2D
{
  // Define the real number values & related functions to be used
#if S2_DOUBLE_PRECISION
  typedef double Real;
  #define s2Sqrt(x) sqrt(x);
#else
  typedef float Real;
  #define s2Sqrt(x) sqrtf(x);
#endif


  // Physics constants
  const Real G = 9.80665;


}

namespace s2 = Spring2D;


#endif // __SETTINGS_H__

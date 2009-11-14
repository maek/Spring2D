#ifndef __SETTINGS_H__
#define __SETTINGS_H__

// Include all the external headers
#include <iostream>
#include <cmath>


// Create the Spring2D namespace & define a shorter synonym for it
namespace Spring2D
{
  // Define the real number values & related functions to be used
#ifdef S2_DOUBLE_PRECISION
  typedef double Real;
  #define s2Sqrt(x) sqrt(x);
#else
  typedef float Real;
  #define s2Sqrt(x) sqrtf(x);
#endif
}

namespace s2 = Spring2D;


#endif // __SETTINGS_H__

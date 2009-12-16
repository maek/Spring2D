#ifndef __SETTINGS_H__
#define __SETTINGS_H__

// Include all the external headers
#include <iostream>
#include <cmath>
#include <list>
#include <set>
#include <queue>
#include <algorithm>
#include <cassert>

// Double precision flag
#define S2_DOUBLE_PRECISION 0


// Create the Spring2D namespace & define a shorter synonym for it
namespace Spring2D
{
  // Define the real number values & related functions to be used
#if S2_DOUBLE_PRECISION
  typedef double Real;
  #define s2sqrt(x)       sqrt(x)
  #define s2hypot(x, y)   hypot(x, y)
  #define s2sin(x)        sin(x)
  #define s2cos(x)        cos(x)
  #define s2atan2(x, y)   atan2(x, y)
  #define s2fabs(x)       fabs(x)
#else
  typedef float Real;
  #define s2sqrt(x)       sqrtf(x)
  #define s2hypot(x, y)   hypotf(x, y)
  #define s2sin(x)        sinf(x)
  #define s2cos(x)        cosf(x)
  #define s2atan2(x, y)   atan2f(x, y)
  #define s2fabs(x)       fabsf(x)
#endif


  // Physics constants
  const Real G = 9.80665;

  // Class declarations (to decrease dependencies)
  class AABB;
  class Body;
  class BroadPhaseDetector;
  class CircleShape;
  class CollisionDetector;
  class Complex;
  class Contact;
  class Engine;
  class Environment;
  class Force;
  class ForceGravity;
  class ForceRegister;
  class Matrix2x2;
  class NarrowPhaseDetector;
  class PolygonShape;
  class RectShape;
  class Shape;
  class Simplex;
  class Torque;
  class TorqueMotor;
  class TorqueRegister;
  class UGrid;
  class Vector2;

}


namespace s2 = Spring2D;


#endif // __SETTINGS_H__

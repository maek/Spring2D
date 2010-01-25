#ifndef __SETTINGS_H__
#define __SETTINGS_H__

// Include all the external headers
#include <iostream>
#include <list>
#include <queue>
#include <cmath>
#include <cfloat>
#include <cassert>

// Double precision flag
#define S2_DOUBLE_PRECISION 0


// Create the Spring2D namespace & define a shorter synonym for it
namespace Spring2D
{
  // Define the real number values & related functions to be used
#if S2_DOUBLE_PRECISION

  typedef double Real;

  const double INFINITE       = DBL_MAX;
  const double EPSILON_TOL    = DBL_EPSILON;
  const double EPSILON_ABS    = 0.0000000000001;
  const double EPSILON_REL    = 0.001;
  const double EPSILON_REL_2  = 0.000001;

  #define s2sqrt(x)             sqrt(x)
  #define s2pow(x, y)           pow(x, y)
  #define s2hypot(x, y)         hypot(x, y)
  #define s2sin(x)              sin(x)
  #define s2cos(x)              cos(x)
  #define s2atan2(x, y)         atan2(x, y)
  #define s2copysign(x, y)      copysign(x, y)
  #define s2fabs(x)             fabs(x)

#else

  typedef float Real;

  const float INFINITE        = FLT_MAX;
  const float EPSILON_TOL     = FLT_EPSILON;
  const float EPSILON_ABS     = 0.00005f;
  const float EPSILON_REL     = 0.01f;
  const float EPSILON_REL_2   = 0.0001f;

  #define s2sqrt(x)             sqrtf(x)
  #define s2pow(x, y)           powf(x, y)
  #define s2hypot(x, y)         hypotf(x, y)
  #define s2sin(x)              sinf(x)
  #define s2cos(x)              cosf(x)
  #define s2atan2(x, y)         atan2f(x, y)
  #define s2copysign(x, y)      copysignf(x, y)
  #define s2fabs(x)             fabsf(x)

#endif


  // Utility functions
  #define s2sqr(x)              (x) * (x)


  // Physics constants
  const Real G = 9.80665;


  // Class declarations (to decrease dependencies)
  class AABB;
  class Body;
  class BroadPhaseDetector;
  class CircleShape;
  class CollisionDetector;
  class CollisionSolver;
  class Complex;
  class Constraint;
  class ConstraintsRegister;
  class Contact;
  class DynamicEntry;
  class DynamicsRegister;
  class Engine;
  class Environment;
  class GenericForce;
  class GenericTorque;
  class Matrix2x2;
  class NarrowPhaseDetector;
  class PolygonShape;
  class RectShape;
  class Shape;
  class Simplex;
  class SpringForce;
  class UGrid;
  class Vector2;

}


namespace s2 = Spring2D;


#endif // __SETTINGS_H__

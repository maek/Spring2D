/*
 * Copyright (C) 2010   Marco Dalla Via (maek@paranoici.org)
 *
 *  This file is part of Spring2D.
 *
 *  Spring2D is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Spring2D is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU Lesser Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser Public License
 *  along with Spring2D. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SETTINGS_H__
#define __SETTINGS_H__

// Include all the external headers
#include <iostream>
#include <list>
#include <vector>
#include <algorithm>
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
  #define s2floor(x)            floor(x)

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
  #define s2floor(x)            floorf(x)

#endif


  // Utility functions
  #define s2sqr(x)              (x) * (x)


  const Real MOTION_THRESHOLD = 5.0;
  const Real MOTION_BIAS      = 0.9;



  // Class declarations (to decrease dependencies)
  class AABR;
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
  class Grid;
  class Matrix2x2;
  class NarrowPhaseDetector;
  class PolygonShape;
  class RectShape;
  class SAP;
  class Shape;
  class Simplex;
  class SpringForce;
  class Vector2;

}


namespace s2 = Spring2D;


#endif // __SETTINGS_H__

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

#include "../include/s2Math.h"


namespace Spring2D
{
  // Canonical vectors
  const Vector2 Vector2::ZERO(0, 0);
  const Vector2 Vector2::X(1, 0);
  const Vector2 Vector2::Y(0, 1);
  const Vector2 Vector2::XY(1, 1);

  // Canonical orientations
  const Complex Complex::ZERO(1, 0);
  const Complex Complex::PI_4(s2sqrt(2) / 2., s2sqrt(2) / 2.);
  const Complex Complex::PI_2(0, 1);
  const Complex Complex::PI(-1, 0);

}

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

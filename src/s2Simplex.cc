#include "../include/s2Simplex.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Compute the point of minimum norm for the simplex & automatically reduce it
  Vector2 Simplex::getPointOfMinimumNorm ()
  {
    if (dimension_ == 0) // single point (1 vertex)
    {
      std::cout << "dimension = 0\n";
      std::cout << "P[0] = " << P[0] << "\n";
      return P[0];
    }


    else if (dimension_ == 1) // segment (2 vertices)
    {
      std::cout << "dimension = 1\n";
      std::cout << "P[0] = " << P[0] << "\n";
      std::cout << "P[1] = " << P[1] << "\n";
      Vector2 A = P[0];
      Vector2 B = P[1];
      Vector2 O(0 ,0);
      Vector2 AB = B - A;

      // Project O onto AB, but deferring the division
      Real numerator = dotProduct(O - A, AB);
      // If O is outside segment & on the A side
      if (numerator < 0)
      {
        // Remove B & return A
        dimension_--;
        return A;
      }
      Real denominator = dotProduct(AB, AB);
      // If O is outside segment & on the B side
      if (numerator > denominator)
      {
        // Remove A & return B
        P[0] = P[1];
        dimension_--;
        return B;
      }
      // O is on the segment
      // Return the projection on AB
      return (A + (numerator / denominator) * AB);
    }


    else // triangle (3 vertices)
    {
      std::cout << "dimension = 2\n";
      std::cout << "P[0] = " << P[0] << "\n";
      std::cout << "P[1] = " << P[1] << "\n";
      std::cout << "P[2] = " << P[2] << "\n";
      Vector2 A = P[0];
      Vector2 B = P[1];
      Vector2 C = P[2];
      Vector2 O(0 ,0);


      // Check if O is in vertex region outside A
      Vector2 AB = B - A;
      Vector2 AC = C - A;
      Vector2 AO = O - A;
      Real d1 = dotProduct(AB, AO);
      Real d2 = dotProduct(AC, AO);
      if (d1 <= 0.0 && d2 <= 0.0)
      {
        // Remove B and C & return A
        dimension_ -= 2;
        return A;
      }

      // Check if O is in vertex region outside B
      Vector2 BO = O - B;
      Real d3 = dotProduct(AB, BO);
      Real d4 = dotProduct(AC, BO);
      if (d3 >= 0.0 && d4 <= d3)
      {
        // Remove A and C & return B
        P[0] = P[1];
        dimension_ -= 2;
        return B;
      }

      // Check if O is in vertex region outside C
      Vector2 CO = O - C;
      Real d5 = dotProduct(AB, CO);
      Real d6 = dotProduct(AC, CO);
      if (d6 >= 0.0 && d5 <= d6)
      {
        // Remove A and B & return C
        P[0] = P[2];
        dimension_ -= 2;
        return C;
      }


      // Check if O is in edge region of AB
      Real vc = d1 * d4 - d3 * d2;
      if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
      {
        // Remove C & return the projection on AB
        dimension_ -= 1;
        Real v = d1 / (d1 - d3);
        return (A + v * AB);
      }

      // Check if O is in edge region of AC
      Real vb = d5 * d2 - d1 * d6;
      if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
      {
        // Remove B & return the projection on AC
        P[1] = P[2];
        dimension_ -= 1;
        Real w = d2 / (d2 - d6);
        return (A + w * AC);
      }

      // Check if O is in edge region of BC
      Real va = d3 * d6 - d5 * d4;
      if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
      {
        // Remove A & return the projection on BC
        P[0] = P[1];
        P[1] = P[2];
        dimension_ -= 1;
        Real w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (B + w * (C - B));
      }


      // O is inside face region
      // Return the projection on ABC
      hasOrigin_ = true;
      Real denominator = 1.0 / (va + vb + vc);
      Real v = vb * denominator;
      Real w = vc * denominator;
      return (A + AB * v + AC * w);
    }

  }


}

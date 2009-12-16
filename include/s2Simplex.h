#ifndef __SIMPLEX_H__
#define __SIMPLEX_H__

#include "s2Settings.h"
#include "s2Math.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The simplex for the GJK
  class Simplex
  {
    public:

      Vector2 P[3];


    public:

      // Constructor
      Simplex () : dimension_(-1), hasOrigin_(false) { }


      // Add a vertex to the simplex
      void expand (const Vector2& VERTEX)
      {
        assert(dimension_ < 2);
        P[++dimension_] = VERTEX;
      }

      // Test if the given vertex is in the simplex
      bool hasVertex (const Vector2& VERTEX)
      {
        for (int i = 0; i <= dimension_; ++i)
        {
          if (VERTEX == P[i])
            return true;
        }

        return false;
      }

      // Test if the simplex contains the origin
      bool hasOriginInside ()
      {
        return hasOrigin_;
      }


      Vector2 getPointOfMinimumNorm ();


    private:

      int dimension_;

      bool hasOrigin_;

  };


}


#endif // __SIMPLEX_H__

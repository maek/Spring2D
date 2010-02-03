#ifndef __SAP_H__
#define __SAP_H__

#include "s2Settings.h"
#include "s2Math.h"
#include "s2Body.h"
#include "s2Shape.h"
#include "s2AABR.h"
#include "s2BroadPhaseDetector.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // The element for the 2 axes lists
  // TODO: make it static (only for this file)
  class Element
  {
    public:

      const Body*       body;

      ConstBodyVector   overlappingList;


      const Real*       min[2];

      const Real*       max[2];


    public:

      // Constructor
      Element (const Body* BODY,
          const Real* MIN1, const Real* MAX1,
          const Real* MIN2, const Real* MAX2)
        : body(BODY)
      {
        min[0] = MIN1;
        min[1] = MIN2;
        max[0] = MAX1;
        max[1] = MAX2;
      }

  };



  bool xElementCompare (Element*, Element*);

  bool yElementCompare (Element*, Element*);



  // ---------------------------------------------------------------------------
  // The element list
  typedef std::vector<Element*> ElementVector;



  void insertionSort (ElementVector&, const int);





  // ---------------------------------------------------------------------------
  // The Sweep And Prune collision detection
  class SAP : public BroadPhaseDetector
  {
    public:

      void findCollisions (const BodyList&, ContactList*);


    private:

      ElementVector xAxis_;

      ElementVector yAxis_;

  };


}


#endif // __SAP_H__

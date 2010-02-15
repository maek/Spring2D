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

#include "../include/s2CollisionSolver.h"


namespace Spring2D
{
  // ---------------------------------------------------------------------------
  // Solve collisions
  void CollisionSolver::solveCollisions (ContactList* contacts,
      bool constraint)
  {
    // Early out
    if (contacts->empty())
    {
      return;
    }

    // Preprocess contacts
    preprocessContacts(contacts);

    // Preprocess constraints data
    if (constraint == true)
    {
      preprocessConstraintsContacts(contacts);
    }


    // Solve interpenetration
    for (ContactList::iterator contactI = contacts->begin();
        contactI != contacts->end(); ++contactI)
    {
      solveInterpenetration(*contactI);
    }


    // Solve velocities
    for (ContactList::iterator contactI = contacts->begin();
        contactI != contacts->end(); ++contactI)
    {
      solveVelocity(*contactI);
    }

  }



  // ---------------------------------------------------------------------------
  // Preprocess the Contact list
  void CollisionSolver::preprocessContacts ( ContactList* contacts)
  {
    for (ContactList::iterator contactI = contacts->begin();
        contactI != contacts->end(); ++contactI)
    {
      // Swap if needed
      if ((*contactI)->body[0]->isStatic())
      {
        (*contactI)->swap();
      }

      // Awake the first body if needed
      if ((*contactI)->body[0]->isSleeping() == true &&
          (*contactI)->body[1]->isSleeping() == false)
      {
        (*contactI)->body[0]->awake();
      }

      // Awake the second body if needed
      if ((*contactI)->body[0]->isSleeping() == false &&
          (*contactI)->body[1]->isSleeping() == true &&
          (*contactI)->body[1]->isStatic() == false)
      {
        (*contactI)->body[1]->awake();
      }


      // Calculate the normal
      (*contactI)->normal = ((*contactI)->point[1] - (*contactI)->point[0]);
      (*contactI)->normal.normalize();

      // Calculate the tangent
      (*contactI)->tangent = (*contactI)->normal.getPerpendicularCopy();


      // Calculate the restitution coefficient
      (*contactI)->restitution =
        ((*contactI)->body[0]->getElasticity() +
         (*contactI)->body[1]->getElasticity()) * 0.5;

      // Calculate the friction coefficient
      (*contactI)->friction =
        ((*contactI)->body[0]->getFriction() +
         (*contactI)->body[1]->getFriction()) * 0.5;


      // Calculate the relative points
      (*contactI)->relativeContactPoint[0] = (*contactI)->point[0] -
        (*contactI)->body[0]->getPosition();
      if ((*contactI)->body[1]->isStatic() == false)
      {
        (*contactI)->relativeContactPoint[1] = (*contactI)->point[1] -
          (*contactI)->body[1]->getPosition();
      }

      Vector2 velocity =
        (*contactI)->body[0]->getVelocity() +
        (*contactI)->body[0]->getRotation() *
        (*contactI)->relativeContactPoint[0].getPerpendicularCopy();
      if ((*contactI)->body[1]->isStatic() == false)
      {
        velocity -=
          (*contactI)->body[1]->getVelocity() +
          (*contactI)->body[1]->getRotation() *
          (*contactI)->relativeContactPoint[1].getPerpendicularCopy();
      }

      // Calculate the closing velocity
      (*contactI)->closingVelocity = dot(velocity, (*contactI)->normal);

      // Calculate the sliding velocity
      (*contactI)->slidingVelocity = dot(velocity, (*contactI)->tangent);


      // Calculate the linear inertia
      (*contactI)->linearInertia[0] = (*contactI)->body[0]->getInverseMass();
      if ((*contactI)->body[1]->isStatic() == false)
      {
        (*contactI)->linearInertia[1] = (*contactI)->body[1]->getInverseMass();
      }

      // Calculate the angular inertia
      Real tcross = cross(
          (*contactI)->relativeContactPoint[0],
          (*contactI)->normal);
      (*contactI)->angularInertia[0] = tcross * tcross *
        (*contactI)->body[0]->getInverseMomentOfInertia();
      if ((*contactI)->body[1]->isStatic() == false)
      {
        tcross = cross(
            (*contactI)->relativeContactPoint[1],
            (*contactI)->normal);
        (*contactI)->angularInertia[1] = tcross * tcross *
          (*contactI)->body[1]->getInverseMomentOfInertia();
      }
    }

  }



  // ---------------------------------------------------------------------------
  // Preprocess the constraints contacts list
  void CollisionSolver::preprocessConstraintsContacts (ContactList* contacts)
  {
    for (ContactList::iterator contactI = contacts->begin();
        contactI != contacts->end(); ++contactI)
    {
      // Set to zero the restitution coefficient
      (*contactI)->restitution = 0;

      // Set to zero the friction coefficient
      (*contactI)->friction = 0;

      // Set to zero the sliding velocity
      (*contactI)->slidingVelocity = 0;
    }

  }



  // ---------------------------------------------------------------------------
  // Solve interpenetrations
  void CollisionSolver::solveInterpenetration (Contact* contact)
  {
    Real totalInertia = contact->linearInertia[0] + contact->angularInertia[0];

    if (contact->body[1]->isStatic() == false)
    {
      totalInertia += contact->linearInertia[1] + contact->angularInertia[1];
    }

    Real iTotalInertia  = (1. / totalInertia);


    contact->body[0]->setPosition(
        contact->body[0]->getPosition() +
        contact->penetrationDepth * contact->linearInertia[0] * iTotalInertia *
        contact->normal);

    contact->body[0]->setOrientation(
        contact->body[0]->getOrientation().rotate(
          s2copysign(1., cross(contact->relativeContactPoint[0], contact->normal)) *
          contact->penetrationDepth * contact->angularInertia[0] * iTotalInertia /
          contact->relativeContactPoint[0].getMagnitude()));


    if (contact->body[1]->isStatic() == false)
    {
      contact->body[1]->setPosition(
          contact->body[1]->getPosition() +
          -contact->penetrationDepth * contact->linearInertia[1] * iTotalInertia *
          contact->normal);

      contact->body[1]->setOrientation(
          contact->body[1]->getOrientation().rotate(
            s2copysign(1., cross(contact->relativeContactPoint[1], contact->normal)) *
            -contact->penetrationDepth * contact->angularInertia[1] * iTotalInertia /
            contact->relativeContactPoint[1].getMagnitude()));
    }
  }



  // ---------------------------------------------------------------------------
  // Solve velocities
  void CollisionSolver::solveVelocity (Contact* contact)
  {
    // Check if the body is still colliding
    if (contact->closingVelocity >= 0)
    {
      return;
    }

    // Remove velocity caused by only acceleration (gravity)
    Real accelerationVelocity =
      dot(contact->body[0]->getVelocityFromAcceleration(), contact->normal);

    if (contact->body[1]->isStatic() == false)
    {
      accelerationVelocity -=
        dot(contact->body[1]->getVelocityFromAcceleration(), contact->normal);
    }

    // Calculate the normal impulse
    Real Jnorm =
      -(1 + contact->restitution) * (contact->closingVelocity - accelerationVelocity) /
      (
       contact->linearInertia[0] +
       contact->angularInertia[0]
      );
    if (contact->body[1]->isStatic() == false)
    {
      Jnorm =
        -(1 + contact->restitution) * (contact->closingVelocity - accelerationVelocity) /
        (
         contact->linearInertia[0] +
         contact->linearInertia[1] +
         contact->angularInertia[0] +
         contact->angularInertia[1]
        );
    }


    // Calculate the angular inertia on the tangent
    Real angularInertia[2];
    Real tcross = cross(
        contact->relativeContactPoint[0],
        contact->tangent);
    angularInertia[0] = tcross * tcross *
      contact->body[0]->getInverseMomentOfInertia();
    if (contact->body[1]->isStatic() == false)
    {
      tcross = cross(
          contact->relativeContactPoint[1],
          contact->tangent);
      angularInertia[1] = tcross * tcross *
        contact->body[1]->getInverseMomentOfInertia();
    }

    // Calculate the tangent impulse
    Real Jtang =
      -contact->slidingVelocity /
      (
       contact->linearInertia[0] +
       angularInertia[0]
      );
    if (contact->body[1]->isStatic() == false)
    {
      Jtang =
        -contact->slidingVelocity /
        (
         contact->linearInertia[0] +
         contact->linearInertia[1] +
         angularInertia[0] +
         angularInertia[1]
        );
    }

    // Clamp if dynamic friction
    if (s2fabs(Jtang) > Jnorm * contact->friction)
    {
      Jtang = s2copysign(Jnorm * contact->friction, Jtang);
    }

    Vector2 J = Jnorm * contact->normal + Jtang * contact->tangent;


    // Apply the impulse (velocity)
    contact->body[0]->setVelocity(contact->body[0]->getVelocity() +
        J * contact->body[0]->getInverseMass());

    // Apply the impulse (rotation)
    contact->body[0]->setRotation(contact->body[0]->getRotation() +
        cross(contact->relativeContactPoint[0], J) *
        contact->body[0]->getInverseMomentOfInertia());


    if (contact->body[1]->isStatic() == false)
    {
      // Apply the impulse (velocity)
      contact->body[1]->setVelocity(contact->body[1]->getVelocity() -
          J * contact->body[1]->getInverseMass());
      // Apply the impulse (rotation)
      contact->body[1]->setRotation(contact->body[1]->getRotation() -
          cross(contact->relativeContactPoint[1], J) *
          contact->body[1]->getInverseMomentOfInertia());
    }

  }


}

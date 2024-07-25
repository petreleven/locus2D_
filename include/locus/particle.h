#pragma once
#include "core.h"
#include "precision.h"
#include "raylib.h"

using locus::real;
using locus::Vector3;
class Particle {
public:
  using Vector3 = locus::Vector3;
  Vector3 position;
  Vector3 velocity;
  Vector3 acceleration;
  real damping;
  Vector3 forceAccum;

protected:
  real inverseMass;

public:
  Particle();
  Particle(Vector3 pos, Vector3 vel, real dampping, real mass);
  void setInverseMass(real _inverseMass) { inverseMass = _inverseMass; }
  void setMass(real _mass) { inverseMass = (1 / _mass); }
  void setVelocity(real x, real y, real z);
  void setAcceleration(real x, real y, real z);
  void setDamping(real);
  void setPosition(real x, real y, real z);
  void clearAccumulator();
  void integrate(real dt);
  void render();
};

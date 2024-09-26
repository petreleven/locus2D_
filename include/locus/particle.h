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
  Color c;


protected:
  real inverseMass;
  real mass;

public:
  Particle();
  Particle(Vector3 pos, Vector3 vel, real dampping, real mass, Color c);
  void setInverseMass(real _inverseMass);
  void setMass(real _mass);
  void setVelocity(real x, real y, real z);
  void setAcceleration(real x, real y, real z);
  void setDamping(real);
  void setPosition(real x, real y, real z);
  void clearAccumulator();
  void integrate(real dt);
  void addForce(const Vector3 &force);
  void setColor(Color c);
  real radius = 5;
  real getMass();
  real getInverseMass() const;
  void render();
};

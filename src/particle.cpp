#include "../include/locus/particle.h"
#include "core.h"
#include <raylib.h>
Particle::Particle() {
  char *a = "Locus";
  SetWindowTitle(a);
}
Particle::Particle(Vector3 pos, Vector3 vel, real dampping, real mass) {
  position = pos;
  velocity = vel;
  damping = dampping;
  setInverseMass(mass);
}
void Particle::integrate(real dt) {

  position += velocity * dt;
  // position += acceleration * dt * dt *0.5;
  // a =a + f/m;
  Vector3 resolvedAcc = acceleration;
  resolvedAcc.addScaledVector(forceAccum, inverseMass);
  // v = v +a *t
  velocity.addScaledVector(resolvedAcc, dt);
  // add drag
  velocity *= damping;
}

void Particle::setVelocity(real x, real y, real z) {
  velocity = Vector3(x, y, z) * 3780;
}
void Particle::setAcceleration(real x, real y, real z) {
  acceleration = Vector3(x, y, z);
}

void Particle::setDamping(real damp) { damping = damp; }

void Particle::setPosition(real x, real y, real z) {
  position = Vector3(x, y, z);
}

void Particle::render() { DrawCircle(position.x, position.y, 5, ORANGE); }
void Particle::clearAccumulator() { forceAccum = Vector3(0.f, 0.f, 0.f); }

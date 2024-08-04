#include "../include/locus/pcontacts.h"
#include <cstddef>
#include <iostream>
#include <math.h>

void ParticleContact::resolve(locus::real dt) {

  resolveVelocity(dt);
  resolvePenetration(dt);
}

locus::real ParticleContact::calculateSeparatingV() {
  // Vseparating_b4_contact = (v0 - v1) . normalContact;
  locus::Vector3 relativeVelocity = p[0]->velocity;
  if (p[1]) {
    relativeVelocity -= p[1]->velocity;
  }
  locus::real vs = relativeVelocity.dotProduct(contactNormal);
  return vs;
}

void ParticleContact::resolveVelocity(locus::real dt) {
  locus::real vs_b4_contact = calculateSeparatingV();

  // no need to resolve collision
  if (vs_b4_contact > 0) {
    return;
  }
  locus::real vs_after_contact = -vs_b4_contact * restitution;
  // RESTING
  locus::Vector3 accCausedVelocity = p[0]->acceleration;
  if (p[1]) {
    accCausedVelocity -= p[1]->acceleration;
  }
  real accCausedSepVelocity = accCausedVelocity.dotProduct(contactNormal) * dt;
  // vel is due to v build up
  if (fabs(accCausedSepVelocity) < 0.001f) {
    vs_after_contact += accCausedSepVelocity * restitution;
    if (vs_after_contact < 0.001f) {
      vs_after_contact = 0.f;
    }
  }
  locus::real deltaVs = vs_after_contact - vs_b4_contact;
  locus::real totalInverseMass = p[0]->getInverseMass();
  if (p[1]) {
    totalInverseMass += p[1]->getInverseMass();
  }
  if (totalInverseMass <= 0) {
    return;
  }
  locus::real impulse = deltaVs / totalInverseMass;
  locus::Vector3 impulseV = contactNormal * impulse;

  // p0_vel = p0_vel + impulse / p0_mass
  p[0]->velocity += impulseV * p[0]->getInverseMass();
  // p1_vel = p1_vel - impulse / p1_mass
  if (p[1]) {

    p[1]->velocity -= impulseV * p[1]->getInverseMass();
  }
}

void ParticleContact::resolvePenetration(real dt) {
  if (penetration <= 0.001f)
    return;
  real totalInverseMass = p[0]->getInverseMass();
  if (p[1]) {
    totalInverseMass += p[1]->getInverseMass();
  }
  if (totalInverseMass <= 0.0f) {
    return;
  }
  // movement_v =normal * -pen * 1/1/(m1 + m2)
  locus::Vector3 movementPen = contactNormal * (penetration / totalInverseMass);
  locus::Vector3 move_0 = movementPen * p[0]->getInverseMass();
  move[0] = move_0;
  p[0]->position += move[0];
  if (p[1]) {
    locus::Vector3 move_1 = movementPen * p[1]->getInverseMass() * -1;
    move[1] = move_1;
    p[1]->position += move[1];
  }
}

ParticleCollisionReSolver::ParticleCollisionReSolver(size_t numParticles) {
  ParticleCollisionReSolver::iterations = numParticles;
  ParticleCollisionReSolver::iterationsUsed = 0;
}
void ParticleCollisionReSolver::solveCollision(
    ParticleContact *particleContacts, size_t size, real dt) {

  iterationsUsed = 0;
  if (particleContacts == nullptr)
    return;
  if (!size)
    return;
  while (iterationsUsed < iterations) {
    locus::real maxSeperatingV = real_INFINITY;
    int maxIndex = 0;
    for (size_t i = 0; i < size; i++) {
      real sepV = particleContacts[i].calculateSeparatingV();
      if (sepV < maxSeperatingV) {
        maxSeperatingV = sepV;
        maxIndex = i;
      }
    }
    particleContacts[maxIndex].resolve(dt);
    locus::Vector3 *movement = particleContacts[maxIndex].move;

    for (size_t i = 0; i < size; i++) {
      if (i == maxIndex)
        return;
      if (particleContacts[i].p[0] == particleContacts[maxIndex].p[0]) {
        particleContacts[i].penetration -=
            (movement[0]).dotProduct(particleContacts[i].contactNormal);
      }

      if (particleContacts[i].p[0] == particleContacts[maxIndex].p[1]) {
        particleContacts[i].penetration -=
            (movement[1]).dotProduct(particleContacts[i].contactNormal);
      }

      if (particleContacts[i].p[1]) {
        if (particleContacts[i].p[1] == particleContacts[maxIndex].p[0]) {
          particleContacts[i].penetration +=
              (movement[0]).dotProduct(particleContacts[i].contactNormal);
        }
        if (particleContacts[i].p[1] == particleContacts[maxIndex].p[0]) {
          particleContacts[i].penetration +=
              (movement[1]).dotProduct(particleContacts[i].contactNormal);
        }
      }
    }

    iterationsUsed++;
  }
}

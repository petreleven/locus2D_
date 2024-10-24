#include "../include/locus/forcegenerator.h"

using namespace locus;
ParticleForceRegistry::ParticleForceRegistry() {}

void ParticleForceRegistry::add(Particle *particle,
                                ParticleForceGenerator *fg) {
  registrations.push_back(SingleRegister(particle, fg));
}

void ParticleForceRegistry::remove(Particle *particle,
                                   ParticleForceGenerator *fg) {
  for (auto p = registrations.begin(); p != registrations.end(); p++) {
    if (p->particle == particle && p->fg == fg) {
      registrations.erase(p);
      break;
    }
  }
}
void ParticleForceRegistry::clear() { registrations.clear(); }

void ParticleForceRegistry::updateForces(real duration) {
  for (Register::iterator i = registrations.begin(); i != registrations.end();
       i++) {
    i->fg->updateForce(i->particle, duration);
  }
}

ParticleGravity::ParticleGravity(const locus::Vector3 &gravity) {
  ParticleGravity::gravity = gravity;
}
void ParticleGravity::updateForce(Particle *particle, real duration) {
  particle->addForce(gravity * particle->getMass());
}

ParticleDrag::ParticleDrag(real k1, real k2) {
  ParticleDrag::k1 = k1;
  ParticleDrag::k2 = k2;
}
void ParticleDrag::updateForce(Particle *particle, real duration) {
  locus::Vector3 normal = (particle->velocity);
  real magnitude = normal.magnitude();
  normal.normalize();
  // -n( k1*|n| + k2*|n||n| )
  locus::Vector3 drag =
      (normal * -1) * (k1 * magnitude + k2 * magnitude * magnitude);

  particle->addForce(drag);
}

ParticleAnchoredSpring::ParticleAnchoredSpring(real restLength,
                                               real springConstant,
                                               Particle *other) {
  ParticleAnchoredSpring::restLength = restLength;
  ParticleAnchoredSpring::springConstant = springConstant;
  ParticleAnchoredSpring::other = other;
}

void ParticleAnchoredSpring::updateForce(Particle *particle, real duration) {
  // hookes law f = -k (dl)
  locus::Vector3 difference = particle->position - other->position;
  real magnitudeDiff = difference.magnitude();
  locus::Vector3 normal = (difference);
  normal.normalize();
  real changeInLength = magnitudeDiff - restLength;

  locus::Vector3 force = normal * (-springConstant * changeInLength);
  particle->addForce(force);
}

ParticleBouyancy::ParticleBouyancy(real s_maxDepth, real h_liquidHeight,
                                   real volumeOfObject, real p_density = 1000) {
  ParticleBouyancy::s_maxDepth = s_maxDepth;
  ParticleBouyancy::h_liquidHeight = h_liquidHeight;
  ParticleBouyancy::p_density = p_density;
  ParticleBouyancy::v_volumeOfObject = volumeOfObject;
}

void ParticleBouyancy::updateForce(Particle *particle, real duration) {
  /*
   ----------------
                   |s_depth
   ----------------height
                   |s_depth
   ----------------
  */
  real depth = particle->position.y;
  if (depth <= (h_liquidHeight - s_maxDepth)) {
    return;
  }
  locus::Vector3 force;
  // FULLY SUBMERGED
  // p = m/v
  if (depth >= (h_liquidHeight + s_maxDepth)) {
    force = locus::Vector3(0.f, -p_density * v_volumeOfObject, 0.f);
  } else {
    // PARTIALY SUMBERGED
    force = locus::Vector3(0.f,
                           -p_density * v_volumeOfObject *
                               (depth - h_liquidHeight - s_maxDepth) /
                               (2 * s_maxDepth),
                           0.f);
  }
  particle->addForce(force);
}

ParticleFakeSpring::ParticleFakeSpring(real springConstant, real damping,
                                       Particle *anchor) {
  ParticleFakeSpring::springConstant = springConstant;
  ParticleFakeSpring::damping = damping;
  ParticleFakeSpring::anchor = anchor;
}
void ParticleFakeSpring::updateForce(Particle *particle, real duration) {
  // DAMPED HARMONIC/ STIFF SPRING
  duration += 0.025f;
  locus::Vector3 po = particle->position - anchor->position;
  locus::Vector3 p_vo = particle->velocity;
  real Y = 0.5 * real_sqrtf(4 * springConstant - damping * damping);
  if (std::abs(Y) <= 0.0001f) {
    return;
  }
  locus::Vector3 c = po * (damping / (2.f * Y)) + p_vo * (1.0f / Y);
  // predicted position
  locus::Vector3 pt =
      (po * real_cosf(Y * duration) + c * real_sinf(Y * duration)) *
      real_expf(-0.5 * duration * damping);

  // calculate acc
  //[(pt - po) * 1/(t*t)  ] -  p_v

  locus::Vector3 acceleration =
      (pt - po) * (1.f / (duration * duration)) - p_vo * duration;

  particle->addForce(acceleration * particle->getMass());
}

ParticleSpring::ParticleSpring(real restLength, real springConstant,
                               real damping) {
  ParticleSpring::restLength = restLength;
  ParticleSpring::springConstant = springConstant;
  ParticleSpring::damping = damping;
}

void ParticleSpring::updateForce(Particle *particleAB[2], real duration) {
  // hookes law f = k(x)
  locus::Vector3 difference = particleAB[0]->position - particleAB[1]->position;
  real magnitudeDiff = difference.magnitude();
  locus::Vector3 normal = difference;
  normal.normalize();
  real changeInLength = magnitudeDiff - restLength;
  real forceScalar = springConstant * changeInLength;
  locus::Vector3 forceAlongNormal = normal * forceScalar;
  particleAB[0]->addForce(forceAlongNormal * -1);
  particleAB[1]->addForce(forceAlongNormal);
  // velocity update
  real totalInverseMass =
      particleAB[0]->getInverseMass() + particleAB[1]->getInverseMass();
  if (totalInverseMass <= 0) {
    return;
  }
  locus::Vector3 vAB = particleAB[0]->velocity - particleAB[1]->velocity;
  real vrel = vAB.dotProduct(normal);
  real newRel = vrel * expf(-damping * duration);
  real vrelDelta = newRel - vrel;
  vrelDelta /= totalInverseMass;
  particleAB[0]->velocity +=
      normal * vrelDelta * particleAB[0]->getInverseMass();
  particleAB[1]->velocity -=
      normal * vrelDelta * particleAB[1]->getInverseMass();
}

void SpringForceRegistry::add(Particle *a, Particle *b) {
  SingleRegister sr = SingleRegister(a, b);
  registrations.push_back(sr);
}
void SpringForceRegistry::add(Particle *a, Particle *b, real restLength) {
  SingleRegister sr = SingleRegister(a, b, restLength);
  registrations.push_back(sr);
}
void SpringForceRegistry::remove(Particle *a, Particle *b) {
  for (auto iterator = registrations.begin(); iterator != registrations.end();
       iterator++) {
    if (a == iterator->a && b == iterator->b) {
      registrations.erase(iterator);
      break;
    }
  }
}
void SpringForceRegistry::updateForces(real dt) {
  Particle *targets[2];
  for (auto p = registrations.begin(); p != registrations.end(); p++) {
    Particle *a = p->a;
    Particle *b = p->b;
    targets[0] = a;
    targets[1] = b;
    p->fg.updateForce(targets, dt);
  }
}

#pragma once
#include "core.h"
#include "particle.h"
#include "plinks.h"
#include "precision.h"
#include <cstddef>
#include <vector>
class MassAggregate;

class ParticleContact {
public:
  Particle *p[2] = {nullptr,  nullptr};
  real restitution;
  locus::Vector3 contactNormal;
  real penetration;
  locus::Vector3 move[2];
  void resolve(locus::real dt);
  locus::real calculateSeparatingV();

protected:
  void resolvePenetration(locus::real dt);

private:
  void resolveVelocity(locus::real dt);
};

class ParticleEdgeContact {
public:
  real penetration;
  locus::Vector3 normal;
  Particle *particleColliding;
  MassAggregate *shape;
  size_t edgeParticleIndices[2];
  locus::Vector3 move[3];

public:
  void resolve(real dt);
};

class ParticleEdgeCollisionDetection {
private:
  bool LineInterSection(locus::Vector3 line1Start, locus::Vector3 line1End,
                        locus::Vector3 line2Start,
                        locus::Vector3 line2End) const;
  std::pair<real, locus::Vector3> closestEdge(Particle &collidingParticle,
                                              locus::Vector3 line2Start,
                                              locus::Vector3 line2End) const;

  bool checkEdgeEdgeCollision(locus::Vector3 &edgeStart,
                              locus::Vector3 &edgeEnd,
                              locus::Vector3 &collidingParticleposition,
                              locus::Vector3 &particlePrevPos,
                              locus::Vector3 &edgecollisionNormal,
                              real &edgeCollisionPenetration) const;

public:
  std::vector<ParticleEdgeContact>
  createContacts(Particle &collidingParticle,
                 std::vector<MassAggregate *> shapes);
};

class ParticleCollisionReSolver {
private:
  unsigned iterations;
  unsigned iterationsUsed;

public:
  ParticleCollisionReSolver(size_t numParticles);
  void solveCollision(ParticleContact *particleContacts, size_t size, real dt);
};

class ParticleEdgeCollisionReSolver {
private:
  unsigned iterations;
  unsigned iterationsUsed;

public:
  ParticleEdgeCollisionReSolver(size_t iterations);
  void solveCollision(std::vector<ParticleEdgeContact> &edgecontacts, real dt);
};

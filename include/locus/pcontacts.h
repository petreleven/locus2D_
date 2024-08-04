#include "core.h"
#include "particle.h"
#include "precision.h"
#include <cstddef>

class ParticleContact {
public:
  Particle *p[2];
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

class ParticleCollisionReSolver{
private:
  unsigned iterations;
  unsigned iterationsUsed;
public:
  ParticleCollisionReSolver(size_t numParticles);
  void solveCollision(ParticleContact *particleContacts, 
                      size_t size,
                      real dt);

};

#pragma once
#include "../app.h"
#include "/home/peter/locus2D/include/locus/forcegenerator.h"
#include "/home/peter/locus2D/include/locus/particle.h"
#include "/home/peter/locus2D/include/locus/plinks.h"

#include <raylib.h>
#include <vector>
using namespace locus;
class BridgesV2 : public Application {
  bool addingParticle;
public:
  MassAggregate shape;
  std::vector<ParticleSpring> springForces;
  std::vector<ParticleSpring> diagonalSpringForces;
  std::vector<std::pair<Particle *, Particle *>> diagonalPairs;
  Particle *first = nullptr;
  Particle *second = nullptr;
  BridgesV2();
  void physicsUpdate(locus::real dt);
  void update() override;
  void startDraw() override;
  void endDraw() override;
  void key(int button) override;
  void mouse() override;
  void setAcceleration(locus::Vector3 gravity);
};

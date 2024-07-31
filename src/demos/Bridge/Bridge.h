#pragma once
#include "../app.h"
#include "forcegenerator.h"
#include "particle.h"
#include <raylib.h>
using namespace locus;
class Bridge : public Application {
private:
  const static unsigned maxPoints = 6;
  locus::real suspensionLength = 60.f;
  locus::real startPlaneX = 200.f;
  locus::real startPlaneY = 400.f;

public:
  Particle points[maxPoints];
  Bridge(locus::ParticleForceRegistry &registery);
  void physicsUpdate(locus::real dt);
  void update() override;
  void startDraw() override;
  void endDraw() override;
  void key(int button) override;
  void mouse() override;
};

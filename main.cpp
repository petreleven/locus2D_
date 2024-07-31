#include "include/locus/core.h"
#include "include/locus/forcegenerator.h"
#include "include/locus/particle.h"
#include "include/locus/precision.h"
#include "particle.h"
#include "raylib.h"
#include "src/demos/Ballistic/Ballistics.h"
#include "src/demos/Bridge/Bridge.h"
#include "src/demos/Fireworks/Fireworks.h"
#include "src/demos/app.h"
#include <cmath>
#include <iostream>
#include <string>
#include <unordered_map>
using namespace locus;
int screenWidth = 800;
int screenHeight = 300;
locus::Vector3 gravity = locus::Vector3(0.f, 300.f, 0.f);
void drag(Particle &base, bool &isdragging);

std::unordered_map<Particle *, bool> draggingStates;
int main() {
  ParticleForceRegistry registry;
  Bridge app = Bridge(registry);
  bool isdragging = false;
  while (!WindowShouldClose()) {
    app.startDraw();
    float dt = GetFrameTime();
    if (dt > 0.016f) {
      registry.updateForces(dt);
      app.physicsUpdate(dt);
    }
    for (Particle &p : app.points) {
      drag(p, isdragging);
    }
    app.update();
    app.endDraw();
  }
}

void drag(Particle &base, bool &isDragging) {
  Vector2 mousePos = GetMousePosition();
  real dragForce = 17000.f;
  locus::Vector3 offset;

  if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
    if (std::abs(base.position.x - mousePos.x) < 50 &&
        std::abs(base.position.y - mousePos.y) < 50) {
      draggingStates[&base] = true;
    }
  } else {
    draggingStates[&base] = false;
    base.setColor(ORANGE);
  }

  if (draggingStates[&base]) {
    base.setColor(PURPLE);
    offset = locus::Vector3(mousePos.x - base.position.x,
                            mousePos.y - base.position.y, 0.f);
    offset.normalize();
    base.addForce(offset * dragForce);
  }
}

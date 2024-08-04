#include "include/locus/core.h"
#include "include/locus/particle.h"
#include "include/locus/pcontacts.h"
#include "particle.h"
#include "raylib.h"
#include "src/demos/Ballistic/Ballistics.h"
#include "src/demos/app.h"
#include <cstddef>
#include <unordered_map>
using namespace locus;
int screenWidth = 800;
int screenHeight = 300;
locus::Vector3 gravity = locus::Vector3(0.f, 1000.f, 0.f);
void drag(Particle &base, bool &isdragging);

std::unordered_map<Particle *, bool> draggingStates;
int main() {
  Application app = Application();
  bool isdragging = false;
  const size_t numParticle = 4;
  Particle allParticles[numParticle] = {
      Particle(locus::Vector3(400, 0, 0), locus::Vector3(0, 0, 0), 0.8f, 1,
               Color{250, 239, 93, 255}),
      Particle(locus::Vector3(400, 400, 0), locus::Vector3(0, 0, 0), 0.8f,
               1, Color{160, 21, 62, 255}),
      Particle(locus::Vector3(460, 430, 0), locus::Vector3(0, 0, 0), 0.8f,
               1, Color{93, 14, 65, 255}),
      Particle(locus::Vector3(520, 400, 0), locus::Vector3(0, 0, 0), 0.8f,
               1, Color{160, 21, 62, 255}),
  };
  ParticleContact pairs[1000];
  allParticles[0].setAcceleration(0.f, gravity.y, 0.f);
  ParticleCollisionReSolver resolver = ParticleCollisionReSolver(12);
  int pause = 0;
  while (!WindowShouldClose()) {
    if (IsKeyPressed(KEY_SPACE)) {
      pause = pause == 0 ? 1 : 0;
    }
    app.startDraw();
    float dt = GetFrameTime();
    if (dt > 0.016f && pause == 0) {
      dt = 0.016f;
      int created = 0;
      // FIND INTERSECTING BODIES
      for (unsigned i = 0; i < numParticle; i++) {
        for (unsigned j = i + 1; j < numParticle; j++) {
          //---START CHECK ---//
          if ((allParticles[i].position - allParticles[j].position)
                      .magnitude() -
                  (2 * 30) <
              0.f) {
            ParticleContact a;
            a.p[0] = &allParticles[i];
            a.p[1] = &allParticles[j];
            a.contactNormal =
                (allParticles[i].position - allParticles[j].position);
            a.contactNormal.normalize();
            a.restitution = .56f; //.56f;
            a.penetration =
                (2 * 30) - (allParticles[i].position - allParticles[j].position)
                               .magnitude();
            pairs[created] = a;
            created++;
          }
          //---END CHECK ---//
        }
      }

      // SOLVE COLLISIONS
      resolver.solveCollision(pairs, created, dt);
      // SOLVE EULER
      for (auto &p : allParticles) {
        p.integrate(dt);
      }
    }

    for (Particle &p : allParticles) {
      p.render();
    }
    drag(allParticles[0], isdragging);
    app.update();
    app.endDraw();
  }
}

void drag(Particle &base, bool &isDragging) {
  Vector2 mousePos = GetMousePosition();
  real dragForce = 100000.f;
  locus::Vector3 offset;

  float offsett = 3.f;
  if (IsKeyDown(KEY_RIGHT)) {
    base.addForce(locus::Vector3(1000, 0, 0));
  }
  if (IsKeyDown(KEY_LEFT)) {
    base.addForce(locus::Vector3(-1000, 0, 0));
  }
  if (IsKeyDown(KEY_UP)) {
    base.position.y = 0.f;
    base.position.x = 400.f;
    base.setVelocity(0, 0, 0);
    base.setAcceleration(0, gravity.y, 0);
  }
  if (IsKeyDown(KEY_DOWN)) {
    base.position.y += offsett;
  }
}

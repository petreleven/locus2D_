#include "Bridge.h"

Bridge::Bridge(locus::ParticleForceRegistry &registery)
    : Application(800, 800, (char *)"LOCUS 2D>spring bridge") {
  locus::real damping = 0.8f;
  locus::real mass = 1.f;
  // 0, 1, 2, 3, 4, 5
  for (unsigned i = 0; i < maxPoints - 2; i++) {
    Particle one = Particle(locus::Vector3(startPlaneX + i * suspensionLength,
                                           startPlaneY, 0.f), // pos
                            locus::Vector3(0.f, 0.f, 0.f),    // vel
                            damping,                          // damping
                            mass,                             // mass
                            BLACK);
    points[i] = one;
  }
  Particle anchor1 = Particle(locus::Vector3(startPlaneX - suspensionLength,
                                             startPlaneY - 2.f, 0.f), // pos
                              locus::Vector3(0.f, 0.f, 0.f),          // vel
                              damping,                                // damping
                              mass,                                   // mass
                              BLACK);
  Particle anchor2 = Particle(
      locus::Vector3(points[maxPoints - 3].position.x + suspensionLength,
                     startPlaneY - 2.f, 0.f), // pos
      locus::Vector3(0.f, 0.f, 0.f),          // vel
      damping,                                // damping
      mass,                                   // mass
      BLACK);
  points[maxPoints - 2] = anchor1;
  points[maxPoints - 1] = anchor2;
  // 0 1 2 3 4 5
  ParticleGravity *g = new ParticleGravity(locus::Vector3(0.f, 600.f, 0.f));
  locus::real K = 200.f;
  for (unsigned i = 0; i < maxPoints - 3; i++) {
    ParticleSpring *springForce =
        new ParticleSpring(suspensionLength, K, &points[i]);
    registery.add(&points[i + 1], springForce);
    ParticleSpring *springForceBackWard =
        new ParticleSpring(suspensionLength, K, &points[i + 1]);
    registery.add(&points[i], springForceBackWard);
    registery.add(&points[i], g);
  }
  // 3rd last point also affected by gravity
  registery.add(&points[maxPoints - 3], g);
  // Suspension between 2nd last point and first point
  ParticleSpring *springForceSusp1 =
      new ParticleSpring(suspensionLength, K, &points[maxPoints - 2]);

  registery.add(&points[0], springForceSusp1);
  // Suspension between  last point and 3rd last point
  ParticleSpring *springForceSusp2 =
      new ParticleSpring(suspensionLength, K, &points[maxPoints - 1]);
  registery.add(&points[maxPoints - 3], springForceSusp2);
}

void Bridge::update() {
  for (Particle &p : points) {
    p.render();
  }
  for (unsigned i = 0; i < maxPoints - 1 - 2; i++) {
    DrawLine(points[i].position.x, points[i].position.y,
             points[i + 1].position.x, points[i + 1].position.y, WHITE);
  }
  DrawLine(points[0].position.x, points[0].position.y,
           points[maxPoints - 2].position.x, points[maxPoints - 2].position.y,
           WHITE);
  DrawLine(points[maxPoints - 3].position.x, points[maxPoints - 3].position.y,
           points[maxPoints - 1].position.x, points[maxPoints - 1].position.y,
           WHITE);
};
void Bridge::physicsUpdate(locus::real dt) {
  for (Particle &p : points) {
    p.integrate(dt);
  }
}

void Bridge::startDraw() { Application::startDraw(); };
void Bridge::endDraw() { Application::endDraw(); };
void Bridge::key(int button) {}
void Bridge::mouse() {}

#include "include/locus/core.h"
#include "include/locus/forcegenerator.h"
#include "include/locus/particle.h"
#include "include/locus/pcontacts.h"
#include "include/locus/plinks.h"
#include "particle.h"
#include "raylib.h"
#include "src/demos/Ballistic/Ballistics.h"
#include "src/demos/BridgesV2/BridgesV2.h"
#include "src/demos/app.h"
#include <cstddef>
#include <iostream>
#include <unordered_map>
#include <vector>
using namespace locus;
int screenWidth = 800;
int screenHeight = 800;
int pause = 0;
bool ismousedragging = false;

void mousedrag(Particle &base, Vector2 &lastMousePos);
locus::Vector3 gravity = locus::Vector3(0.f, 100.f, 0.f);
void drag(Particle &base);
std::vector<ParticleContact>
groundCollision(std::vector<MassAggregate *> &shapes, unsigned screenHeight,
                unsigned screenwidth);

std::unordered_map<Particle *, bool> draggingStates;
MassAggregate shape = MassAggregate();
MassAggregate shape2 = MassAggregate();

Particle a = Particle(locus::Vector3(200, 400, 0), locus::Vector3(0, 0, 0), 0.8,
                      100, GRAY);
Particle b = Particle(locus::Vector3(250, 400, 0), locus::Vector3(0, 0, 0), 0.8,
                      100, GRAY);
Particle c = Particle(locus::Vector3(350, 400, 0), locus::Vector3(0, 0, 0), 0.8,
                      100, GRAY);
Particle d = Particle(locus::Vector3(350, 450, 0), locus::Vector3(0, 0, 0), 0.8,
                      100, GRAY);
Particle e = Particle(locus::Vector3(250, 450, 0), locus::Vector3(0, 0, 0), 0.8,
                      100, GRAY);
Particle f = Particle(locus::Vector3(250, 500, 0), locus::Vector3(0, 0, 0), 0.8,
                      100, GRAY);
Particle g = Particle(locus::Vector3(350, 500, 0), locus::Vector3(0, 0, 0), 0.8,
                      100, GRAY);

Particle h = Particle(locus::Vector3(350, 550, 0), locus::Vector3(0, 0, 0), 0.8,
                      100, GRAY);

Particle i = Particle(locus::Vector3(250, 550, 0), locus::Vector3(0, 0, 0), 0.8,
                      100, GRAY);

Particle j = Particle(locus::Vector3(250, 600, 0), locus::Vector3(0, 0, 0), 0.8,
                      100, GRAY);

Particle k = Particle(locus::Vector3(350, 600, 0), locus::Vector3(0, 0, 0), 0.8,
                      100, GRAY);

Particle l = Particle(locus::Vector3(350, 650, 0), locus::Vector3(0, 0, 0), 0.8,
                      100, GRAY);

Particle m = Particle(locus::Vector3(250, 650, 0), locus::Vector3(0, 0, 0), 0.8,
                      100, GRAY);
Particle n = Particle(locus::Vector3(200, 650, 0), locus::Vector3(0, 0, 0), 0.8,
                      100, GRAY);

Particle o = Particle(locus::Vector3(200, 500, 0), locus::Vector3(0, 0, 0), 0.8,
                      100, GRAY);

Particle a2 = Particle(locus::Vector3(250, 100, 0), locus::Vector3(0, 0, 0),
                       0.8, 100, GRAY);
Particle b2 = Particle(locus::Vector3(350, 100, 0), locus::Vector3(0, 0, 0),
                       0.8, 100, GRAY);
Particle c2 = Particle(locus::Vector3(350, 200, 0), locus::Vector3(0, 0, 0),
                       0.8, 100, GRAY);
Particle d2 = Particle(locus::Vector3(250, 200, 0), locus::Vector3(0, 0, 0),
                       0.8, 100, GRAY);
Particle e2 = Particle(locus::Vector3(200, 150, 0), locus::Vector3(0, 0, 0),
                       0.8, 100, GRAY);

int main() {
  BridgesV2 app = BridgesV2();
  shape.addParticle(a);
  shape.addParticle(b);
  shape.addParticle(c);
  shape.addParticle(d);
  shape.addParticle(e);
  shape.addParticle(f);
  shape.addParticle(g);
  shape.addParticle(h);
  shape.addParticle(i);
  shape.addParticle(j);
  shape.addParticle(k);
  shape.addParticle(l);
  shape.addParticle(m);
  shape.addParticle(n);
  shape.addParticle(o);

  shape2.addParticle(a2);
  shape2.addParticle(b2);
  shape2.addParticle(c2);
  shape2.addParticle(d2);
  shape2.addParticle(e2);
  shape2.setAcceleration(gravity);
  shape.setAcceleration(gravity);
  shape.color = RED;
  shape2.color = ORANGE;
  std::vector<MassAggregate *> staticshapes{&shape};
  std::vector<MassAggregate *> dynamicshapes{&shape2}; //&app.shape};
  std::vector<MassAggregate *> allobjects = {&shape, &shape2};
  ParticleCollisionReSolver particleresolver = ParticleCollisionReSolver(20);
  ParticleEdgeCollisionDetection detector = ParticleEdgeCollisionDetection();
  ParticleEdgeCollisionReSolver edgeresolver =
      ParticleEdgeCollisionReSolver(100);
  SpringForceRegistry springregistry;
  springregistry.add(&shape.particles[0], &shape.particles[1]);
  springregistry.add(&shape.particles[1], &shape.particles[2]);
  springregistry.add(&shape.particles[2], &shape.particles[3]);
  springregistry.add(&shape.particles[3], &shape.particles[4]);
  springregistry.add(&shape.particles[4], &shape.particles[5]);
  springregistry.add(&shape.particles[5], &shape.particles[6]);
  springregistry.add(&shape.particles[6], &shape.particles[7]);
  springregistry.add(&shape.particles[7], &shape.particles[8]);
  springregistry.add(&shape.particles[8], &shape.particles[9]);
  springregistry.add(&shape.particles[9], &shape.particles[10]);
  springregistry.add(&shape.particles[10], &shape.particles[11]);
  springregistry.add(&shape.particles[11], &shape.particles[12]);
  springregistry.add(&shape.particles[12], &shape.particles[13]);
  springregistry.add(&shape.particles[13], &shape.particles[14]);

  springregistry.add(&shape.particles[0], &shape.particles[4]);
  springregistry.add(&shape.particles[1], &shape.particles[3]);
  springregistry.add(&shape.particles[2], &shape.particles[4]);
  springregistry.add(&shape.particles[5], &shape.particles[7]);
  springregistry.add(&shape.particles[6], &shape.particles[8]);
  springregistry.add(&shape.particles[9], &shape.particles[11]);
  springregistry.add(&shape.particles[10], &shape.particles[12]);
  springregistry.add(&shape.particles[9], &shape.particles[13]);
  springregistry.add(&shape.particles[4], &shape.particles[14]);
  springregistry.add(&shape.particles[5], &shape.particles[14]);
  springregistry.add(&shape.particles[8], &shape.particles[14]);
  springregistry.add(&shape.particles[9], &shape.particles[14]);

  // shape2
  springregistry.add(&shape2.particles[0], &shape2.particles[1]);
  springregistry.add(&shape2.particles[1], &shape2.particles[2]);
  springregistry.add(&shape2.particles[2], &shape2.particles[3]);
  springregistry.add(&shape2.particles[3], &shape2.particles[0]);
  springregistry.add(&shape2.particles[3], &shape2.particles[4]);
  springregistry.add(&shape2.particles[4], &shape2.particles[0]);
  ParticleCable cable(300, 0.8, shape2.particles[3], shape.particles[0]);

  Vector2 lastMousePos = GetMousePosition();
  while (!WindowShouldClose()) {
    app.startDraw();
    if (IsKeyPressed(KEY_SPACE)) {
      pause = pause == 0 ? 1 : 0;
    }
    float dt = GetFrameTime();

    app.update();
    // physics update
    if (dt > 0.016f && pause == 0) {
      // dt = 0.016f;
      app.physicsUpdate(dt);
      std::vector<ParticleEdgeContact> contacts;

      for (Particle &p : shape.particles) {
        auto genContacts = detector.createContacts(p, dynamicshapes);
        for (auto c : genContacts) {
          contacts.push_back(c);
        }
      }
      for (Particle &p : shape2.particles) {
        auto genContacts = detector.createContacts(p, staticshapes);
        for (auto c : genContacts) {
          contacts.push_back(c);
        }
      }

      // particle particle collisions
      std::vector<ParticleContact> contactsShtoSh;
      const real collisionThreshold = 1e-6f;
      if (true) {
        for (Particle &p1 : shape.particles) {
          p1.radius = 2.1f;
          for (Particle &p2 : shape2.particles) {
            p2.radius = 2.1f;
            // Calculate future positions
            locus::Vector3 p1Future = p1.position + p1.velocity * dt;
            locus::Vector3 p2Future = p2.position + p2.velocity * dt;

            // Check current and future positions
            locus::Vector3 relativePos = p1.position - p2.position;
            locus::Vector3 relativeFuturePos = p1Future - p2Future;

            real distanceSquared = relativePos.squareMagnitude();
            real futureDictanceSquared = relativeFuturePos.squareMagnitude();
            real collisionDistanceSquared =
                (p1.radius + p2.radius) * (p1.radius + p2.radius);

            if (distanceSquared <
                    collisionDistanceSquared + collisionThreshold ||
                futureDictanceSquared <
                    collisionDistanceSquared + collisionThreshold) {

              // Use the smaller distance for contact resolution
              real distance =
                  std::sqrt(std::min(distanceSquared, futureDictanceSquared));
              real penetration = p1.radius + p2.radius - distance;

              locus::Vector3 contactNormal =
                  (distanceSquared < futureDictanceSquared) ? relativePos
                                                            : relativeFuturePos;
              contactNormal.normalize();

              ParticleContact contact;
              contact.contactNormal = contactNormal;
              contact.p[0] = const_cast<Particle *>(&p1);
              contact.p[1] = const_cast<Particle *>(&p2);
              contact.penetration = penetration;
              contact.restitution = 0.5f;

              contactsShtoSh.push_back(contact);
            }
          }
        }
        particleresolver.solveCollision(contactsShtoSh.data(),
                                        contactsShtoSh.size(), dt);
      }
      if (true) {
        auto groundcontacts = groundCollision(allobjects, 800, 800);
        particleresolver.solveCollision(groundcontacts.data(),
                                        groundcontacts.size(), dt);
      }
      edgeresolver.solveCollision(contacts, dt);
      springregistry.updateForces(dt);
      shape.update(dt);
      shape2.update(dt);
    }
    // normalupdate
    drag(shape2.particles[0]);
    mousedrag(shape2.particles[0], lastMousePos);
    // app.update();
    shape.render();
    shape2.render();
    app.endDraw();
  }
}

std::vector<ParticleContact>
groundCollision(std::vector<MassAggregate *> &shapes, unsigned screenHeight,
                unsigned screenWidth) {
  screenHeight = 700;
  std::vector<ParticleContact> groundContacts;
  for (auto shape : shapes) {
    for (size_t i = 0; i < shape->particles.size(); i++) {
      auto &p = shape->particles[i];
      if (p.position.y > screenHeight) {
        ParticleContact contact;
        contact.contactNormal = locus::Vector3(0.f, -1.0f, 0.f);
        contact.penetration = p.position.y - screenHeight;
        contact.restitution = 0.f;
        contact.p[0] = &p;
        contact.p[1] = NULL;
        groundContacts.push_back(contact);
      }
      if (p.position.x < 0.f) {
        ParticleContact contact;
        contact.contactNormal = locus::Vector3(1.0f, 0.f, 0.f);
        contact.penetration = p.position.x - 0.f;
        contact.restitution = 0.8f;
        contact.p[0] = &p;
        contact.p[1] = NULL;
        groundContacts.push_back(contact);
      } else if (p.position.x > screenWidth) {
        ParticleContact contact;
        contact.contactNormal = locus::Vector3(-1.0f, 0.f, 0.f);
        contact.penetration = p.position.x - screenWidth;
        contact.restitution = 0.2f;
        contact.p[0] = &p;
        contact.p[1] = NULL;
        groundContacts.push_back(contact);
      }
    }
  }
  return groundContacts;
}

void drag(Particle &base) {
  float offsett = 3.f;
  float force = 40000;
  if (IsKeyDown(KEY_RIGHT)) {
    base.addForce(locus::Vector3(force, 0, 0));
  }
  if (IsKeyDown(KEY_LEFT)) {
    base.addForce(locus::Vector3(-force, 0, 0));
    // base.position.x -= offsett;
  }
  if (IsKeyDown(KEY_UP)) {
    base.addForce(locus::Vector3(0, -force, 0));
  }
  if (IsKeyDown(KEY_DOWN)) {
    base.addForce(locus::Vector3(0, force, 0));
  }
}
void mousedrag(Particle &base, Vector2 &lastMousePos) {
  const float force = 20000 * 20;
  if (!IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
    return;
  }
  Vector2 currentMousePos = GetMousePosition();
  Vector2 direction = {currentMousePos.x - lastMousePos.x,
                       currentMousePos.y - lastMousePos.y};
  locus::Vector3 dir = locus::Vector3(direction.x, direction.y, 0.f);
  DrawCircleV({base.position.x, base.position.y}, 20, BLUE);
  dir.normalize();
  base.addForce(dir * force);
  lastMousePos = currentMousePos;
}

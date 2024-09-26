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
Particle a = Particle(locus::Vector3(200, 400, 0), locus::Vector3(0, 0, 0), 0.8,
                      100, GRAY);
Particle b = Particle(locus::Vector3(400, 400, 0), locus::Vector3(0, 0, 0), 0.8,
                      100, GRAY);
Particle c = Particle(locus::Vector3(400, 600, 0), locus::Vector3(0, 0, 0), 0.8,
                      100, GRAY);
Particle d = Particle(locus::Vector3(200, 600, 0), locus::Vector3(0, 0, 0), 0.8,
                      100, GRAY);
MassAggregate shape2 = MassAggregate();
Particle a2 = Particle(locus::Vector3(250, 100, 0), locus::Vector3(0, 0, 0),
                       0.8, 100, GRAY);
Particle b2 = Particle(locus::Vector3(350, 100, 0), locus::Vector3(0, 0, 0),
                       0.8, 100, GRAY);
Particle c2 = Particle(locus::Vector3(350, 200, 0), locus::Vector3(0, 0, 0),
                       0.8, 100, GRAY);
Particle d2 = Particle(locus::Vector3(250, 200, 0), locus::Vector3(0, 0, 0),
                       0.8, 100, GRAY);

int main() {
  BridgesV2 app = BridgesV2();
  shape.width = 200;
  shape.springK = 200;
  shape.damping = 6;
  shape.addParticle(a);
  shape.addParticle(b);
  shape.addParticle(c);
  shape.addParticle(d);
  shape2.width = 100;
  shape2.springK = 200;
  shape2.damping = 6;
  shape2.addParticle(a2);
  shape2.addParticle(b2);
  shape2.addParticle(c2);
  shape2.addParticle(d2);
  shape2.setAcceleration(gravity);
  shape.setAcceleration(gravity);
  shape2.color = ORANGE;
  std::vector<MassAggregate *> staticshapes{&shape};
  std::vector<MassAggregate *> dynamicshapes{&shape2}; //&app.shape};
  std::vector<MassAggregate *> allobjects = {&shape, &shape2};
  ParticleCollisionReSolver particleresolver = ParticleCollisionReSolver(8);
  ParticleEdgeCollisionDetection detector = ParticleEdgeCollisionDetection();
  ParticleEdgeCollisionReSolver edgeresolver =
      ParticleEdgeCollisionReSolver(16 * 10);
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
      dt = 0.016f;
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

      auto groundcontacts = groundCollision(allobjects, 800, 800);
      const size_t size = groundcontacts.size();
      ParticleContact *contactsGen[size];
      for (size_t i = 0; i < size; i++) {
        contactsGen[i] = &groundcontacts[i];
      }
      particleresolver.solveCollision(contactsGen[0], size, dt);

      edgeresolver.solveCollision(contacts, dt);
      shape.update(dt);
      shape2.update(dt);
    }
    // normalupdate
    drag(shape.particles[0]);
    mousedrag(shape.particles[0], lastMousePos);
    // app.update();
    Vector2 p1 =
        Vector2{shape.particles[0].position.x, shape.particles[0].position.y};
    Vector2 p2 =
        Vector2{shape.particles[3].position.x, shape.particles[3].position.y};
    Vector2 p3 =
        Vector2{shape.particles[2].position.x, shape.particles[2].position.y};
    Vector2 *points[] = {&p1, &p2, &p3};
    DrawTriangleFan(points[0], 3, BLUE);
    shape.render();
    shape2.render();
    app.endDraw();
  }
}

std::vector<ParticleContact>
groundCollision(std::vector<MassAggregate *> &shapes, unsigned screenHeight,
                unsigned screenWidth) {
  std::vector<ParticleContact> groundContacts;
  for (auto shape : shapes) {
    for (size_t i = 0; i < shape->particles.size(); i++) {
      auto &p = shape->particles[i];
      if (p.position.y > screenHeight) {
        ParticleContact contact;
        contact.contactNormal = locus::Vector3(0.f, -1.0f, 0.f);
        contact.penetration = p.position.y - screenHeight;
        contact.restitution = 0.2f;
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
  float force = 10000;
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

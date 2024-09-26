#include "BridgesV2.h"
#include <cstddef>
#include <iostream>
#include <raylib.h>
#include <vector>

BridgesV2::BridgesV2()
    : Application(800, 800, (char *)"Locus2D -Bridges using springs") {
  addingParticle = false;
  springForces.push_back(ParticleSpring(10, 20, 0.8));
  shape = MassAggregate();
}

void BridgesV2::startDraw() { Application::startDraw(); }
void BridgesV2::endDraw() { Application::endDraw(); }

void BridgesV2::update() {
  std::vector<Particle> &particles = shape.particles;
  key(0);
  if (particles.size() == 0) {
    return;
  }
  Particle *springParticles[2];
  // normal lines
  for (size_t i = 0; i < particles.size(); i++) {
    unsigned next = (i + 1) % particles.size();
    DrawLineEx({particles[i].position.x, particles[i].position.y},
               {particles[next].position.x, particles[next].position.y}, 10,
               BLACK);
    particles[i].render();
  }
  // diagonal lines
  for (size_t i = 0; i < diagonalPairs.size(); i++) {
    DrawLineEx({diagonalPairs[i].first->position.x,
                diagonalPairs[i].first->position.y},
               {diagonalPairs[i].second->position.x,
                diagonalPairs[i].second->position.y},
               5, WHITE);
  }
  Application::update();
}

void BridgesV2::physicsUpdate(locus::real dt) {

  std::vector<Particle> &particles = shape.particles;
  if (particles.size() == 0) {
    return;
  }

  // integrate
  shape.update(dt);
  for (size_t i = 0; i < particles.size(); i++) {
    particles[i].integrate(dt);
  }
  // force constraints
  Particle *springParticles[2];
  for (size_t i = 0; i < particles.size() - 1; i++) {
    unsigned next = (i + 1) % particles.size();
    springParticles[0] = &particles[i];
    springParticles[1] = &particles[next];
    springForces[i + 1].updateForce(springParticles, dt);
  }
  // force constraints diagonal
  for (size_t i = 0; i < diagonalSpringForces.size(); i++) {
    springParticles[0] = diagonalPairs[i].first;
    springParticles[1] = diagonalPairs[i].second;
    diagonalSpringForces[i].updateForce(springParticles, dt);
  }

  // last two points
  if (particles.size() > 1) {
    springParticles[0] = &particles[0];
    springParticles[1] = &particles[particles.size() - 1];
    springForces[0].updateForce(springParticles, dt);
  }
}

void BridgesV2::key(int _) {
  std::vector<Particle> &particles = shape.particles;
  if (IsKeyPressed(KEY_A)) {
    addingParticle = !addingParticle;
    std::cout << addingParticle << "\n";
  }
  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && addingParticle) {
    Vector2 pos = GetMousePosition();
    Particle x = Particle(locus::Vector3(pos.x, pos.y, 0),
                          locus::Vector3(0, 0, 0), 0.8, 1, GREEN);
    particles.push_back(x);
    if (particles.size() == 1) {
      return;
    }
    unsigned lastParticleIdx = particles.size() - 2;
    ParticleSpring spf =
        ParticleSpring((particles[particles.size() - 1].position -
                        particles[lastParticleIdx].position)
                           .magnitude(),
                       20, 0.8);
    springForces.push_back(spf);
    springForces[0].setRestLength(
        (particles[0].position - particles[particles.size() - 1].position)
            .magnitude());
  }
  // ADD DIAGONAL FORCES MANUALLY
  if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
    addingParticle = false;
    Vector2 pos = GetMousePosition();
    locus::Vector3 pos3D(pos.x, pos.y, 0.f);
    for (size_t i = 0; i < particles.size(); i++) {
      if ((pos3D - particles[i].position).magnitude() <= 10) {
        particles[i].c = RED;
        // cache selected points
        if (first == nullptr) {
          first = &particles[i];
        } else {
          if (first != &particles[i]) {
            second = &particles[i];
          }
        }
        // add cached two points to diagonal  pair
        if (first && second) {
          diagonalPairs.push_back(
              std::pair<Particle *, Particle *>(first, second));
          diagonalSpringForces.push_back(ParticleSpring(
              (first->position - second->position).magnitude(), 20, 0.8));
          first = nullptr;
          second = nullptr;
        }
      }
    }
  }
  if (IsKeyPressed(KEY_F)) {
    setAcceleration(locus::Vector3(0, 100, 0));
  }
}

void BridgesV2::setAcceleration(locus::Vector3 gravity) {
  std::vector<Particle> &particles = shape.particles;
  for (Particle &p : particles) {
    p.setAcceleration(gravity.x, gravity.y, gravity.z);
  }
}

void BridgesV2::mouse() {}

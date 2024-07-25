#include "Ballistics.h"
#include <iostream>
#include <raylib.h>

Ballistic::Ballistic() {
  currentType = PISTOL;
  for (AmmoRound *c = ammo; c < ammo + ammoRounds; c++) {
    c->type = UNUSED;
  }
}
void Ballistic::fire() {
  AmmoRound *a;
  for (a = ammo; a < ammo + ammoRounds; a++) {
    if (a->type == UNUSED)
      break;
  }
  if (a >= ammo + ammoRounds)
    return;

  switch (currentType) {
  case PISTOL:
    a->particle.setMass(2.0f);
    a->particle.setVelocity(35.f, 0.0f, 0.f);
    a->particle.setAcceleration(0.f, 1.f, 0.f);
    a->particle.setDamping(0.99f);
    break;
  case FIREBALL:
    a->particle.setMass(1.0f);
    a->particle.setVelocity(10.f, 0.0f, 0.f);
    a->particle.setAcceleration(0.f, -0.6f, 0.f);
    a->particle.setDamping(0.9f);
    break;
  case LASER:
    a->particle.setMass(0.1);
    a->particle.setVelocity(100.f, 0.0f, 0.f);
    a->particle.setAcceleration(0.f, 0.f, 0.f);
    a->particle.setDamping(0.99f);
    break;
  }
  a->particle.setPosition(40.0f, 100.f, 0.f);
  a->type = currentType;
  a->particle.clearAccumulator();
}
void Ballistic::startDraw() { Application::startDraw(); }
void Ballistic::endDraw() { Application::endDraw(); }

void Ballistic::update() {
  float deltaTime = GetFrameTime();
  if (deltaTime < 0.016f){return;}
  deltaTime *= 0.01;
  for (AmmoRound *c = ammo; c < ammo + ammoRounds; c++) {
    if (c->type == UNUSED) {
      continue;
    }
    c->particle.integrate(deltaTime);
    if (c->particle.position.x > screenWidth ||
        c->particle.position.y > screenHeight) {
      c->type = UNUSED;
    }
  }

  // Draw
  for (AmmoRound *c = ammo; c < ammo + ammoRounds; c++) {
    if (c->type == UNUSED) {
      continue;
    }
    c->particle.render();
  }
  switch (currentType) {
  case PISTOL:
    DrawText("PISTOL",  140,  140, 12, WHITE);
    break;
  case FIREBALL:
    DrawText("FIREBALL", 140, 140, 12, WHITE);
    break;
  case LASER:
    DrawText("LASER", 140, 140, 12, WHITE);
    break;
  }
  Application::update();
}

void Ballistic::key(int btn) {
  if (IsKeyDown(KEY_F1)) {
    currentType = PISTOL;
  }
  if (IsKeyDown(KEY_F2)) {
    currentType = FIREBALL;
  }
  if (IsKeyDown(KEY_F3)) {
    currentType = LASER;
    DrawText("LASER", screenWidth - 40, screenHeight + 40, 12, WHITE);
  }
}
void Ballistic::mouse() {
  if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
    fire();
  }
}

#include "Fireworks.h"
#include <raylib.h>

FireWorkDemo::FireWorkDemo() : Application() {
  for (FireWork *firework = fireworks; firework < fireworks + maxFireworks;
       firework++) {
    firework->type = 0;
  }
  initFireWorkRules();
}
FireWorkDemo::~FireWorkDemo() {}
void FireWorkDemo::startDraw() { Application::startDraw(); }
void FireWorkDemo::endDraw() { Application::endDraw(); }
void FireWorkDemo::deInit() { Application::deInit(); }
void FireWorkDemo::update() {
  float deltaTime = GetFrameTime();

  if (deltaTime >= 0.016f) {
    deltaTime *= 0.1f;
    for (FireWork *firework = fireworks; firework < fireworks + maxFireworks;
         firework++) {
      // SHOULD WE PROCESS THIS
      if (firework->type <= 0) {
        continue;
      }
      // MOVE IT AND CHECK IF IT DISINTEGRATES
      bool remove = firework->update(deltaTime);
      if (remove) {

        firework->type = 0;

        FireWorkRule *rule = &fireworkrules[firework->type];
        for (unsigned i = 0; i < rule->payloadCount; i++) {
          FireWorkRule::Payload *payload = rule->payloads + i;
          create(payload->type, payload->count, firework);
        }
      }
    }
  }
  // rendering

  for (FireWork *firework = fireworks; firework < fireworks + maxFireworks;
       firework++) {
    if (firework->type <= 0) {
      continue;
    }
    firework->render();
  }

  Application::update();
}
void FireWorkDemo::create(unsigned type, unsigned count,
                          const FireWork *parent = nullptr) {
  for (unsigned i = 0; i < count; i++) {
    create(type, parent);
  }
}
void FireWorkDemo::create(unsigned type, const FireWork *parent) {
  FireWorkRule *rule = &fireworkrules[type];
  rule->create(&fireworks[nextFirework], parent);
  nextFirework += 1;
  nextFirework %= maxFireworks;
}

void FireWorkDemo::initFireWorkRules() {
  fireworkrules[0].init(2);
  fireworkrules[0].setParameters(
      1,                                 // type
      1.5f, 0.4f,                        // maxAge minAge,
      locus::Vector3(-5.f, -29.f, 0.0f), // min velocity
      locus::Vector3(5.f, -4.f, 0.0f),   // max velocity
      0.98f                              // damping
  );
  fireworkrules[0].payloads[0].set(3, 15);
  fireworkrules[0].payloads[1].set(4, 10);
  // rule 2
  fireworkrules[1].init(1);
  fireworkrules[1].setParameters(
      2,                                   // type
      0.9f, .4f,                           // maxAge minAge,
      locus::Vector3(-1.8f, -12.9f, 0.0f), // min velocity
      locus::Vector3(1.8f, -2.f, 0.0f),    // max velocity
      0.4f                                 // damping
  );
  fireworkrules[1].payloads[0].set(1, 12);
  // rule3
  fireworkrules[2].init(3);
  fireworkrules[2].setParameters(
      3,                                  // type
      1.9f, .5f,                          // maxAge minAge,
      locus::Vector3(-3.f, -55.9f, 0.0f), // min velocity
      locus::Vector3(5.f, -2.f, 0.0f),    // max velocity
      0.7f                                // damping
  );
  fireworkrules[2].payloads[0].set(2, 12);
  fireworkrules[2].payloads[1].set(3, 10);
  fireworkrules[2].payloads[2].set(1, 10);
  // rule 4
  fireworkrules[3].init(3);
  fireworkrules[3].setParameters(
      4,                                  // type
      2.5f, .2f,                          // maxAge minAge,
      locus::Vector3(1.1f, -12.9f, 0.0f), // min velocity
      locus::Vector3(12.f, -10.f, 0.0f),  // max velocity
      0.7f                                // damping
  );
  fireworkrules[3].payloads[0].set(6, 13);
  fireworkrules[3].payloads[1].set(4, 14);
  fireworkrules[3].payloads[2].set(5, 16);
  // rule 5
  fireworkrules[4].init(0);
  fireworkrules[4].setParameters(
      5,                                    // type
      2.5f, .9f,                            // maxAge minAge,
      locus::Vector3(-10.1f, -20.9f, 0.0f), // min velocity
      locus::Vector3(10.f, -2.f, 0.0f),     // max velocity
      0.7f                                  // damping
  );
  // rule 6
  fireworkrules[5].init(1);
  fireworkrules[5].setParameters(
      6,                                   // type
      2.5f, 0.4f,                          // maxAge minAge,
      locus::Vector3(-5.1f, -11.9f, 0.0f), // min velocity
      locus::Vector3(5.f, -6.f, 0.0f),     // max velocity
      0.7f                                 // damping
  );
  fireworkrules[5].payloads[0].set(3, 13);
}

void FireWorkDemo::key(int btn) {
  if (IsKeyDown(KEY_ZERO)) {
    create(0, nullptr);
  } else if (IsKeyDown(KEY_ONE)) {
    create(1, nullptr);
  } else if (IsKeyDown(KEY_TWO)) {
    create(2, nullptr);
  } else if (IsKeyDown(KEY_THREE)) {
    create(3, nullptr);
  } else if (IsKeyDown(KEY_FOUR)) {
    create(4, nullptr);
  }
}

void FireWorkDemo::mouse() {}

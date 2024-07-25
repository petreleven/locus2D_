#include "../app.h"
#include "particle.h"
#include <raylib.h>
#pragma once
class FireWork : public Particle {
public:
  unsigned type;
  locus::real age;
  // return true if we need to remove it from rendering
  bool update(locus::real duration) {
    integrate(duration);
    age -= duration;
    return (age < 0.001f) || (position.y <= 0.01f);
  }
};
struct FireWorkRule {
  unsigned type;
  locus::real maxAge;
  locus::real minAge;
  locus::Vector3 minVelocity;
  locus::Vector3 maxVelocity;
  locus::real damping;
  // manages how a firework disintegrates
  // eg type and number of fireworks to disintegrate to
  struct Payload {
    unsigned type;
    unsigned count;
    Payload() : count(0) {}
    void set(unsigned type, unsigned count) {
      Payload::type = type;
      Payload::count = count;
    }
  };
  // One rule can have more than one payload
  // i.e the firework managed can disintegrate to more
  // with different  types;
  Payload *payloads;
  unsigned payloadCount;
  FireWorkRule() : payloadCount(0), payloads(nullptr) {}
  void setParameters(unsigned type, locus::real maxAge, locus::real minAge,
                     locus::Vector3 minVelocity, locus::Vector3 maxVelocity,
                     locus::real damping) {
    FireWorkRule::type = type;
    FireWorkRule::maxAge = maxAge;
    FireWorkRule::minAge = minAge;
    FireWorkRule::minVelocity = minVelocity * 100;
    FireWorkRule::maxVelocity = maxVelocity * 100;
    FireWorkRule::damping = damping;
  }
  void init(unsigned count) {
    FireWorkRule::payloadCount = count;
    payloads = new Payload[payloadCount];
  }
  void create(FireWork *firework, const FireWork *parent = nullptr) {
    locus::Vector3 vel;
    locus::Vector3 pos;
    locus::real age;

    if (parent != nullptr) {
      vel = parent->velocity;
      pos = parent->position;
      if (parent->position.y < 0.01f) {
        return;
      }
    } else {
      pos = locus::Vector3(400.f, 790.f, 0.f);
    }
    vel += locus::Vector3::randomVector(minVelocity, maxVelocity);
    firework->velocity = vel;
    firework->position = pos;
    firework->age = locus::Vector3::locusRandom(static_cast<real>(minAge),
                                                static_cast<real>(maxAge));
    firework->type = type;
    firework->setDamping(1);
    firework->setAcceleration(0.f, 100.f, 0.f);
    firework->clearAccumulator();
  }
  ~FireWorkRule() {
    if (payloads != nullptr) {
      delete[] payloads;
    }
  }
};
class FireWorkDemo : public Application {
  const static unsigned maxFireworks = 1024;
  const static unsigned rulesCount = 6;
  FireWork fireworks[maxFireworks];
  FireWorkRule fireworkrules[rulesCount];
  unsigned nextFirework;
  void create(unsigned type, unsigned count, const FireWork *parent);
  void create(unsigned type, const FireWork *parent);
  void initFireWorkRules();

public:
  FireWorkDemo();
  ~FireWorkDemo();
  void startDraw() override;
  void update() override;
  void endDraw() override;
  void deInit() override;
  void key(int button) override;
  void mouse() override;
};

#pragma once
#include "core.h"
#include "particle.h"
#include "precision.h"
#include <raylib.h>
#include <vector>

class ParticleForceGenerator {
public:
  virtual void updateForce(Particle *particle, real duration) = 0;
};

class ParticleGravity : public ParticleForceGenerator {
  locus::Vector3 gravity;

public:
  ParticleGravity(const locus::Vector3 &gravity);
  void updateForce(Particle *particle, real duration) override;
};

class ParticleDrag : public ParticleForceGenerator {
  real k1;
  real k2;

public:
  ParticleDrag(real k1, real k2);
  void updateForce(Particle *particle, real duration) override;
};

class ParticleAnchoredSpring : public ParticleForceGenerator {
private:
  real springConstant;
  real restLength;
  Particle *other;

public:
  ParticleAnchoredSpring() : other(nullptr){};
  ParticleAnchoredSpring(real restLength, real springConstant, Particle *other);
  void updateForce(Particle *particle, real duration) override;
};

class ParticleBouyancy : public ParticleForceGenerator {
  real s_maxDepth;
  real h_liquidHeight;
  real p_density;
  real v_volumeOfObject;

public:
  ParticleBouyancy(real s_maxDepth, real h_liquidHeight, real volumeOfObject,
                   real p_density);
  void updateForce(Particle *particle, real duration) override;
};

class ParticleFakeSpring : public ParticleForceGenerator {
  real damping;
  real springConstant;
  Particle *anchor;

public:
  ParticleFakeSpring(real springConstant, real damping, Particle *anchor);
  void updateForce(Particle *particle, real duration) override;
};

class ParticleSpring {
private:
  real springConstant;
  real restLength;
  real damping;

public:
  ParticleSpring();
  ParticleSpring(real restLength, real springConstant, real damping);
  void updateForce(Particle *particleAB[2], real duration);
  void setRestLength(real length) { restLength = length; }
  void setSpringK(real K) { springConstant = K; }
};

namespace locus {
class ParticleForceRegistry {
  /*HOLDS A SINGLE REGISTER (A FORCE AND PARTICLE IT AFFECTS)*/
protected:
  struct SingleRegister {
    Particle *particle;
    ParticleForceGenerator *fg;
    SingleRegister(Particle *particle, ParticleForceGenerator *fg) {
      SingleRegister::particle = particle;
      SingleRegister::fg = fg;
    }
  };
  typedef std::vector<SingleRegister> Register;
  Register registrations;

public:
  ParticleForceRegistry();
  /* CREATES A NEW REGISTER OF A PAIR */
  void add(Particle *particle, ParticleForceGenerator *fg);
  /* REMOVES A REGISTER PAIR */
  void remove(Particle *particle, ParticleForceGenerator *fg);
  /* CLEARS THE WHOLE REGISTER(connections) BUT DOESNT DELETE PARTICLES OR FGs
   */
  void clear();
  void updateForces(real duration);
};
class SpringForceRegistry {
  struct SingleRegister {
    Particle *a;
    Particle *b;
    ParticleSpring fg = ParticleSpring(10.f, 10.f, 10.f);
    real springK = 100.f;
    real damping = 150.f;
    real restLength = 20.f;
    SingleRegister(Particle *a, Particle *b) {
      SingleRegister::a = a;
      SingleRegister::b = b;
      restLength = (a->position - b->position).magnitude();
      fg = ParticleSpring(restLength, springK, damping);
    }
    SingleRegister(Particle *a, Particle *b, real restLength) {
      SingleRegister::a = a;
      SingleRegister::b = b;
      SingleRegister::restLength = restLength;
      fg = ParticleSpring(restLength, springK, damping);
    }
  };
  typedef std::vector<SingleRegister> Register;
  Register registrations;

public:
  void add(Particle *a, Particle *b);
  void add(Particle *a, Particle *b, real restLength);
  void remove(Particle *a, Particle *b);
  void updateForces(real dt);
};

} // namespace locus

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
} // namespace locus

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

class ParticleSpring : public ParticleForceGenerator {
private:
  real springConstant;
  real restLength;
  Particle *other;

public:
  ParticleSpring(real restLength, real springConstant, Particle *other);
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

class ParticleFakeSpring : public ParticleForceGenerator{
    real damping;
    real springConstant;
    Particle *anchor;
    public:
    ParticleFakeSpring(real springConstant, real damping, Particle *anchor);
    void updateForce(Particle *particle, real duration) override;
};

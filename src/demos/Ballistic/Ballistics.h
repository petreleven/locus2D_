#include "../app.h"
#include "particle.h"
#include <raylib.h>
class Ballistic : public Application {
  enum ProjectileType { UNUSED = 0, PISTOL, FIREBALL, LASER };
  struct AmmoRound {
    Particle particle;
    ProjectileType type;
    void render() { particle.render(); }
  };

  ProjectileType currentType;
  const static unsigned ammoRounds = 16;
  AmmoRound ammo[ammoRounds];
  void fire();

public:
  Ballistic();
  void update() override;
  void startDraw() override;
  void endDraw() override;
  void key(int button) override;
  void mouse() override;
};

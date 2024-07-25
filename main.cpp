#include "include/locus/core.h"
#include "include/locus/particle.h"
#include "particle.h"
#include "raylib.h"
#include "src/demos/Ballistic/Ballistics.h"
#include "src/demos/Fireworks/Fireworks.h"

#include <iostream>
#include <string>
using namespace locus;
int screenWidth = 800;
int screenHeight = 300;
locus::Vector3 gravity = locus::Vector3(0.f, 34000.f, 0.f);
int main() {
  FireWorkDemo c = FireWorkDemo();
  while (!WindowShouldClose()) {
    c.key(1);
    c.startDraw();
    c.update();
    c.endDraw();
  }
}

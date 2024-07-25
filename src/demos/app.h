#pragma once

#include "raylib.h"
class Application {
protected:
  int screenWidth;
  int screenHeight;
  int fps = 60;
  char *title;

public:
  Application();
  Application(int screenWidth, int screenHeight, char *title);
  virtual void startDraw();
  virtual void endDraw();
  virtual void update();
  virtual void deInit();
  virtual void key(int button);
  virtual void mouse();
  virtual int getFps() const;

};

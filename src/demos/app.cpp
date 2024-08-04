#include "app.h"

Application::Application() {
  screenWidth = 800;
  screenHeight = 800;
  char *x = title==nullptr ? (char *)"LOCUS2D" : title;
  InitWindow(screenWidth, screenHeight, x);
  SetTargetFPS(60);
}

Application::Application(int _screenWidth, int _screenHeight, char *_title) {
  screenWidth = _screenWidth;
  screenHeight = _screenHeight;
  title = _title;
  InitWindow(screenWidth, screenHeight, title);
  SetTargetFPS(60);
}
void Application::startDraw() {

  Color background{87, 10, 87, 255};
  BeginDrawing();
  ClearBackground(background);
}
void Application::endDraw() { EndDrawing(); }
void Application::update() {}
void Application::mouse(){}

void Application::deInit() { CloseWindow(); }

void Application::key(int button) {}
int Application::getFps() const { return fps; }

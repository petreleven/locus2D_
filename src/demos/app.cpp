#include "app.h"

Application::Application() {
  screenWidth = 800;
  screenHeight = 800;
  char *x = (char *)"LOCUS2D";
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
  Color background{23, 59, 69, 1};
  BeginDrawing();
  ClearBackground(background);
}
void Application::endDraw() { EndDrawing(); }
void Application::update() {}
void Application::mouse(){}

void Application::deInit() { CloseWindow(); }

void Application::key(int button) {}
int Application::getFps() const { return fps; }

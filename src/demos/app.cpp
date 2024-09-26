#include "app.h"
#include <raylib.h>

Application::Application() {
  screenWidth = 800;
  screenHeight = 800;
  char *x = title == nullptr ? (char *)"LOCUS2D" : title;
  InitWindow(screenWidth, screenHeight, x);
  SetTargetFPS(60);
  mainCamera.target = Vector2{screenWidth / 2.f, screenHeight / 2.f};
  mainCamera.offset = Vector2{screenWidth / 2.f, screenHeight / 2.f};
  mainCamera.rotation = 0.f;
  mainCamera.zoom = 1.0f;
}

Application::Application(int _screenWidth, int _screenHeight, char *_title) {
  screenWidth = _screenWidth;
  screenHeight = _screenHeight;
  title = _title;
  InitWindow(screenWidth, screenHeight, title);
  SetTargetFPS(60);
  mainCamera.target = Vector2{screenWidth / 2.f, screenHeight / 2.f};
  mainCamera.offset = Vector2{screenWidth / 2.f, screenHeight / 2.f};
  mainCamera.rotation = 0.f;
  mainCamera.zoom = 1.0f;
}
void Application::startDraw() {
  BeginDrawing();
  BeginMode2D(mainCamera);
  Color background{48, 52, 70, 255};
  ClearBackground(background);
}
void Application::endDraw() {
  EndMode2D();
  EndDrawing();
}
void Application::update() {}
void Application::mouse() {}

void Application::deInit() { CloseWindow(); }

void Application::key(int button) {}
int Application::getFps() const { return fps; }

add_subdirectory(Raylib/raylib)
#add_library(raylib ./raylib/src/raylib.h)

target_include_directories(raylib PUBLIC "${CMAKE_CURRENT_DIR}/Raylib/raylib")

cmake_minimum_required(VERSION 3.8)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set Clang as the compiler
set(CMAKE_C_COMPILER clang)

set(CMAKE_CXX_COMPILER clang++)

project(locus2D)
# Enable debug symbols
set(CMAKE_BUILD_TYPE Debug)

# Enable Address Sanitizer
# Enable Address Sanitizer and Leak Sanitizer
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address,leak -fno-omit-frame-pointer -g")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address,leak")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

add_subdirectory(include/external)
add_subdirectory(src)
add_subdirectory(src/demos)
add_subdirectory(src/demos/Ballistic)
add_subdirectory(src/demos/Bridge)
add_subdirectory(src/demos/Fireworks)
add_subdirectory(src/demos/BridgesV2)
add_executable(main main.cpp)
target_link_libraries(main PUBLIC raylib core particle precision app Ballistics Fireworks  forcegenerator Bridge pcontacts plinks BridgesV2)
target_include_directories(main PUBLIC
              "${PROJECT_BINARY_DIR}"
               "${PROJECT_SOURCE_DIR}/include/locus"
               "${PROJECT_SOURCE_DIR}/src"
               "${PROJECT_SOURCE_DIR}/src/demos"
               "${PROJECT_SOURCE_DIR}/src/demos/Ballistics"
               "${PROJECT_SOURCE_DIR}/src/demos/Bridge"
               "${PROJECT_SOURCE_DIR}/src/demos/Fireworks"
               "${PROJECT_SOURCE_DIR}/src/demos/BridgeV2"
               "${PROJECT_SOURCE_DIR}/include/external/Raylib/raylib")

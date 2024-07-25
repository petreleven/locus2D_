add_library(Fireworks Fireworks.cpp)

target_include_directories(Fireworks PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(Fireworks PUBLIC
               "${CMAKE_CURRENT_SOURCE_DIR}/../")

target_include_directories(Fireworks PUBLIC
               "${CMAKE_CURRENT_SOURCE_DIR}/....../include/locus")
target_link_libraries(Fireworks PUBLIC app)
target_link_libraries(Fireworks PUBLIC particle)

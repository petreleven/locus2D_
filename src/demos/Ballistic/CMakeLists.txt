add_library(Ballistics Ballistics.cpp)

target_include_directories(Ballistics PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(Ballistics PUBLIC
               "${CMAKE_CURRENT_SOURCE_DIR}/../")

target_include_directories(Ballistics PUBLIC
               "${CMAKE_CURRENT_SOURCE_DIR}/....../include/locus")
target_link_libraries(Ballistics PUBLIC app)
target_link_libraries(Ballistics PUBLIC particle)

add_library(Bridge Bridge.cpp)

target_include_directories(Bridge PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(Bridge PUBLIC
               "${CMAKE_CURRENT_SOURCE_DIR}/../")

target_include_directories(Bridge PUBLIC
               "${CMAKE_CURRENT_SOURCE_DIR}/....../include/locus")
target_link_libraries(Bridge PUBLIC app)
target_link_libraries(Bridge PUBLIC app)
target_link_libraries(Bridge PUBLIC particle)
target_link_libraries(Bridge PUBLIC forcegenerator)

add_library(app app.cpp)




target_include_directories(app PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
                                "${CMAKE_CURRENT_SOURCE_DIR}/..../include/locus")
target_link_libraries(app PUBLIC core  raylib)

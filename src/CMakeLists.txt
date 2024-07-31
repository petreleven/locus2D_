add_library(precision INTERFACE)
add_library(core INTERFACE)
#src file library
add_library(particle particle.cpp)
add_library(forcegenerator forcegenerator.cpp)


# Specify the include directories for the header-only libraries
target_include_directories(precision INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/../include/locus")
target_include_directories(core INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/../include/locus")
#include for the cpp library
target_include_directories(particle PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include/locus")
target_include_directories(forcegenerator PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include/locus")



#linking
target_link_libraries(core  INTERFACE precision )
target_link_libraries(particle PUBLIC core)
target_link_libraries(forcegenerator PUBLIC particle)

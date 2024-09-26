add_library(BridgesV2 BridgesV2.cpp)

target_include_directories(BridgesV2 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(BridgesV2 PUBLIC
               "${CMAKE_CURRENT_SOURCE_DIR}/../")

target_include_directories(BridgesV2 PUBLIC
               "${CMAKE_CURRENT_SOURCE_DIR}/....../include/locus")

target_link_libraries(BridgesV2 PUBLIC app)
target_link_libraries(BridgesV2 PUBLIC particle)
target_link_libraries(BridgesV2 PUBLIC forcegenerator)

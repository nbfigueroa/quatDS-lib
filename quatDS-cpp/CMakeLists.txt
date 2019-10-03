project(quatDS)
cmake_minimum_required(VERSION 2.8)

####################### CMAKE STUFF ##################
set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/test)
set(CMAKE_INSTALL_PREFIX    /usr/)
set(LIBRARY_OUTPUT_PATH     ${PROJECT_SOURCE_DIR}/lib )
set(LIB_INSTALL_PATH        ${CMAKE_INSTALL_PREFIX}lib/) 
set(CMAKE_MODULE_PATH       ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
set(INCLUDE_DIR             ${CMAKE_CURRENT_SOURCE_DIR}/include)    
set(SRC_DIR                 ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -O2")

set(CMAKE_CXX_FLAGS "-std=c++11")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# find_package (Eigen3 3.3 REQUIRED NO_MODULE)


###########
## Build ##
###########


set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(H_FILES     ${INCLUDE_DIR}/Quaternions.h)

## Specify additional locations of header files
include_directories(include ${EIGEN3_INCLUDE_DIRS})

## Declare a C++ library
set(SRCS src/quatDS/Quaternions.cpp)

## Declare a cpp library
add_library(${PROJECT_NAME} ${H_FILES}  ${SRCS})
target_link_libraries(${PROJECT_NAME})

## Declare a C++ executable
add_executable(test_quatDS      src/test_quatDS.cpp)
target_link_libraries(test_quatDS  quatDS)


# INSTALL
message(status "    CMAKE_INSTALL_PREFIX:" ${CMAKE_INSTALL_PREFIX})
install(TARGETS ${PROJECT_NAME} DESTINATION lib EXPORT ${PROJECT_NAME}-targets)
install(EXPORT ${PROJECT_NAME}-targets DESTINATION lib/${PROJECT_NAME})
install(FILES ${SGD_INC} DESTINATION include/)
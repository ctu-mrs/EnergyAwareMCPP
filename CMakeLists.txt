cmake_minimum_required(VERSION 3.1.2)
project(coverage_mission_planner)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
  add_compile_options(/W4 /WX)
else()
  add_compile_options(-Wall -Wextra -pedantic -Werror)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_executable(${PROJECT_NAME} src/main.cpp src/MapPolygon.cpp include/MapPolygon.hpp src/utils.cpp src/algorithms.cpp src/EnergyCalculator.cpp src/ShortestPathCalculator.cpp include/ShortestPathCalculator.hpp include/custom_types.hpp include/mstsp_solver/Target.h src/mstsp_solver/TargetSet.cpp include/mstsp_solver/TargetSet.h include/mstsp_solver/SolverConfig.h src/mstsp_solver/MstspSolver.cpp include/mstsp_solver/MstspSolver.h include/mstsp_solver/Insertion.h include/SimpleLogger.h include/LoggerRos.h)

find_package(yaml-cpp)


target_include_directories(${PROJECT_NAME} PUBLIC include ${YAML_CPP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

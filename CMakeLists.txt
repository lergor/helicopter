cmake_minimum_required(VERSION 3.9)
project(helicopter)

set(CMAKE_CXX_STANDARD 17)

find_package(PythonLibs 2.7)

add_executable(helicopter main.cpp include/matplotlibcpp.h
        src/engine.h src/wind_cursor.h src/vec.h src/Helicopter.h)
target_include_directories(helicopter PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(helicopter ${PYTHON_LIBRARIES})

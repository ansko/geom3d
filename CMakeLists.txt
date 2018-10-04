cmake_minimum_required (VERSION 2.6) 
 
set (PROJECT geometry) 
 
project (${PROJECT}) 

if (CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "\
${CMAKE_CXX_FLAGS} \
-Wall -Wextra \
-Wshadow -Wnon-virtual-dtor \
-pedantic \
-Werror -g -lm -O3 \
-std=c++17")
#https://gist.github.com/lefticus/10191322

endif (CMAKE_COMPILER_IS_GNUCXX)

set (COMMON_HEADERS
    src/point.hpp
    src/line_segment.hpp
    src/plane.hpp
    src/line.hpp
    src/polygon.hpp

    src/pretty_printer.hpp
    src/utils.hpp
)
set (COMMON_SOURCES
    src/point.cpp
    src/line_segment.cpp
    src/plane.cpp
    src/line.cpp
    src/polygon.cpp

    src/pretty_printer.cpp
)


add_executable (my_testing_exe
    ${COMMON_HEADERS} ${COMMON_SOURCES}
     main_geometry.cpp)

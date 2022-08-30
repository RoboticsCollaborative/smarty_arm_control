cmake_minimum_required(VERSION 2.8)
project(smarty_arm_control C CXX)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE true)
set(OS_LIBS pthread rt m)

# SOEM
find_package(soem)

# Add the headers from the <include> directory
include_directories(
        include/
)

# Add the source files from the <src> directory
add_library(${PROJECT_NAME}_lib
        src/rdda_ecat.c
        src/init_AEV.c
        src/rdda_base.c
        src/shm.c
)

target_link_libraries(${PROJECT_NAME}_lib soem ${OS_LIBS})

# Add the executable files
add_executable(${PROJECT_NAME}
        src/rdda_smartyarm.c
        )

# Linking for libraries
target_link_libraries(${PROJECT_NAME}
        ${PROJECT_NAME}_lib
        )


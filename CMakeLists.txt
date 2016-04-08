cmake_minimum_required(VERSION 3.2)
project(EnergyManagementNode)

include(../cmake-xc16/MPLABX.cmake)

set(SOURCES
    src/main.c
    gen/energy_management.c
)

set(HEADERS
    gen/energy_management.h
)

set(NODE_FILES
    energy_management.ncf
)

add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/gen/energy_management.h ${CMAKE_CURRENT_SOURCE_DIR}/gen/energy_management.c
                   COMMAND LIN -t PIC${CMAKE_SYSTEM_PROCESSOR} -i uart1 ${NODE_FILES}
                   DEPENDS ${NODE_FILES}
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(EnergyManagementNode ${SOURCES} ${HEADERS} ${NODE_FILES})
target_compile_definitions(EnergyManagementNode PUBLIC -DFCY=2000000)
target_compile_options(EnergyManagementNode PUBLIC -std=gnu99)
target_include_directories(EnergyManagementNode PUBLIC include gen)
generate_mplabx_project(EnergyManagementNode)

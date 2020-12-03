cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

set(${PROJECT_NAME}_HEADERS
        CandidateFace.h
        CGALTypes.h
#        Draw.h
        Intersection.h
        Optimization.h
        Orientation.h
        Planarity.h
        PlanarSegmentation.h
        Segment.h
        Simplification.h
        StructureGraph.h
        Utils.h
        )

set(${PROJECT_NAME}_SOURCES
        main.cpp
        Planarity.cpp
        PlanarSegmentation.cpp
        Simplification.cpp
        StructureGraph.cpp
        )

add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})

target_include_directories(${PROJECT_NAME} PRIVATE
        ${POLYGONIZATION_scip_DIR}
        ${POLYGONIZATION_scip_DIR}/scip
        ${POLYGONIZATION_rply_DIR}
        )

target_link_libraries( ${PROJECT_NAME} scip soplex rply)

include(../cmake/UseCGAL.cmake)
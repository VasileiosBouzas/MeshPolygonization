cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

set(${PROJECT_NAME}_HEADERS
        CandidateFace.h
        CGALTypes.h
        Intersection.h
        Optimization.h
        Orientation.h
        Planarity.h
        PlanarSegmentation.h
        Segment.h
        Simplification.h
        StructureGraph.h
        Utils.h
        solver/linear_program.h
        solver/linear_program_solver.h
        )

set(${PROJECT_NAME}_SOURCES
        main.cpp
        Planarity.cpp
        PlanarSegmentation.cpp
        Simplification.cpp
        StructureGraph.cpp
        solver/linear_program.cpp
        solver/linear_program_solver.cpp
        solver/linear_program_solver_SCIP.cpp
        solver/linear_program_solver_GUROBI.cpp
        )

add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})

target_include_directories(${PROJECT_NAME} PRIVATE
        ${POLYGONIZATION_scip_DIR}
        ${POLYGONIZATION_scip_DIR}/scip
        ${POLYGONIZATION_rply_DIR}
        )

target_link_libraries( ${PROJECT_NAME} 3rd_scip 3rd_soplex 3rd_rply)

# The resources directory
target_compile_definitions(
        ${PROJECT_NAME}
        PRIVATE
        "POLYGONIZATION_ROOT_DIR=\"${POLYGONIZATION_ROOT}\""
)

include(./cmake/FindGUROBI.cmake)
if (GUROBI_FOUND)
    message(STATUS "Gurobi include dir: " ${GUROBI_INCLUDE_DIRS})
    message(STATUS "Gurobi libraries: " ${GUROBI_LIBRARIES})

    target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_GUROBI)

    target_include_directories(${PROJECT_NAME} PRIVATE ${GUROBI_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${GUROBI_LIBRARIES})
endif()

include(./cmake/UseCGAL.cmake)


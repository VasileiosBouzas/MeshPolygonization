cmake_minimum_required(VERSION 3.1)

################################################################################

project(MeshPolygonization)

################################################################################

# One shouldn't generate the BUILD project directly in the SOURCES folder!
if ( ${CMAKE_CURRENT_BINARY_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR} )
        if ( NOT SAME_BUILD_AND_SOURCE_FOLDER_WARNING_ALREADY_ISSUED )
                message(FATAL_ERROR
                    "It is not advised to BUILD the binaries directly in the SOURCE folder!\n If
                    you want to proceed with this option, just CONFIGURE the project once again" )
                set( SAME_BUILD_AND_SOURCE_FOLDER_WARNING_ALREADY_ISSUED TRUE )
        endif()
endif()

################################################################################

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

################################################################################

find_package(CGAL QUIET)
if (NOT CGAL_FOUND)
    message(WARNING "This project require CGAL but CGAL was not found. Please make sure CGAL is installed, and then"
            "set 'CGAL_DIR' to the directory containing 'CGALConfig.cmake' or 'cgal-config.cmake' to help find CGAL.")
    return()
endif ()

################################################################################

### Configuration
set(POLYGONIZATION_ROOT             ${CMAKE_CURRENT_LIST_DIR})
set(POLYGONIZATION_3RD_PARTY_ROOT   ${POLYGONIZATION_ROOT}/3rd_party)

set(POLYGONIZATION_INCLUDE_DIR      ${POLYGONIZATION_ROOT} ${CMAKE_CURRENT_BINARY_DIR})

set(POLYGONIZATION_scip_DIR         ${POLYGONIZATION_3RD_PARTY_ROOT}/scip)
set(POLYGONIZATION_soplex_DIR       ${POLYGONIZATION_3RD_PARTY_ROOT}/soplex)
set(POLYGONIZATION_rply_DIR         ${POLYGONIZATION_3RD_PARTY_ROOT}/rply)

################################################################################

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY  ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY  ${CMAKE_BINARY_DIR}/lib)

################################################################################

add_subdirectory(3rd_party)
add_subdirectory(Polygonization)
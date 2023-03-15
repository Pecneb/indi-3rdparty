cmake_minimum_required(VERSION 3.0)
PROJECT(DIYGOTO CXX C)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake_modules/")

include(GNUInstallDirs)

set (GENERIC_VERSION_MAJOR 0)
set (GENERIC_VERSION_MINOR 1)

#find_package(CFITSIO REQUIRED)
find_package(INDI REQUIRED)
#find_package(ZLIB REQUIRED)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h )
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/indi_diygoto.xml.cmake ${CMAKE_CURRENT_BINARY_DIR}/indi_diygoto.xml )

include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})
#include_directories( ${CFITSIO_INCLUDE_DIR})

#include(CMakeCommon)

############# GENERIC CCD ###############
set(diygoto_SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/mount_driver.cpp
)

add_executable(indi_diygoto ${diygoto_SRCS})

#target_link_libraries(indi_diygoto ${INDI_LIBRARIES} ${CFITSIO_LIBRARIES} m ${ZLIB_LIBRARY})
#target_link_libraries(indi_diygoto ${INDI_LIBRARIES} ${ZLIB_LIBRARY})
target_link_libraries(indi_diygoto ${INDI_LIBRARIES})
target_link_libraries(indi_diygoto indidriver)

install(TARGETS indi_diygoto RUNTIME DESTINATION bin)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_diygoto.xml DESTINATION ${INDI_DATA_DIR})

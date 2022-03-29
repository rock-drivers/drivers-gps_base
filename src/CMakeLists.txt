set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(GDAL REQUIRED)
add_definitions(${GDAL_CFLAGS})
include_directories(${GDAL_INCLUDE_DIRS})
link_directories(${GDAL_LIBRARY_DIRS})

rock_library(gps_base
    SOURCES UTMConverter.cpp rtcm3.cpp RTCMReassembly.cpp
    HEADERS UTMConverter.hpp BaseTypes.hpp rtcm3.hpp RTCMReassembly.hpp
    DEPS_PKGCONFIG base-types iodrivers_base
)

target_link_libraries(gps_base ${GDAL_LIBRARIES})


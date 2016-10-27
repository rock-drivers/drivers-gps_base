rock_library(gps_base
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(gps_base_bin Main.cpp
    DEPS gps_base)


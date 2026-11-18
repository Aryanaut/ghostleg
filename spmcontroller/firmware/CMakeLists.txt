# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Hardware-in-the-loop: SIM_REPLAY=1 idf.py build replaces the sensor VALUES
# with a replayed Avionics B trajectory while leaving all the sensor TIMING
# real. See main/sim_replay.h.
#
# The value is FORCEd into the cache on every configure, and defaults to 0, so
# that dropping the environment variable actually turns the rig off. The first
# version of this only defined the macro when the variable was present, which
# left the previous definition sitting in the CMake cache: a plain
# `idf.py build` then produced a binary that still had synthetic sensors in it
# and looked, from the command line, exactly like flight firmware. The boot
# banner is the backstop, but do not rely on it - if you are switching modes,
# `idf.py fullclean` first and then check the STATUS line below.
if(DEFINED ENV{SIM_REPLAY})
    set(_sim_replay $ENV{SIM_REPLAY})
else()
    set(_sim_replay 0)
endif()
set(SIM_REPLAY ${_sim_replay} CACHE STRING "1 = hardware-in-the-loop replay, 0 = flight" FORCE)
add_compile_definitions(SIM_REPLAY=${SIM_REPLAY})
if(SIM_REPLAY)
    message(STATUS "*** SIM_REPLAY=1 - SENSOR VALUES ARE SYNTHETIC, THIS IS NOT FLIGHT FIRMWARE ***")
else()
    message(STATUS "SIM_REPLAY=0 - flight firmware, real sensors")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(avionics_c_software)

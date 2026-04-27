cmake_minimum_required(VERSION 3.10)
project(rtklib_ntrip_obs C)

set(CMAKE_C_STANDARD 99)

add_subdirectory(src)
add_subdirectory(ntrip_rtcm_obs)

cmake_minimum_required(VERSION 3.10)
project(rtklib_ntrip_obs C)

set(CMAKE_C_STANDARD 99)

# MSVC parallel compiles sharing one PDB need serialized PDB writes (C1041)
if(MSVC)
    add_compile_options(/FS)
endif()

add_subdirectory(src)
add_subdirectory(ntrip_rtcm_obs)

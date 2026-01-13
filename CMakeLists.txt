cmake_minimum_required(VERSION 3.8)
project(amapfd_soc_mcmf_gbs_astar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
endif()

add_executable(amapfd_soc_mcmf_gbs_astar src/amapfd-soc-mcmf-gbs-astar.cpp)

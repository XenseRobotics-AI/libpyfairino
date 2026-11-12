cmake_minimum_required(VERSION 3.20)
project(fairino_ik LANGUAGES CXX)
find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)
if(DEFINED ENV{FAIRINO_PINOCCHIO_PREFIX})
  set(PIN_PREFIX "$ENV{FAIRINO_PINOCCHIO_PREFIX}")
else()
  execute_process(
    COMMAND "${Python_EXECUTABLE}" -c
      "import pathlib,sys; print(next(pathlib.Path(p)/'cmeel.prefix' for p in sys.path if p and (pathlib.Path(p)/'cmeel.prefix/include/pinocchio/multibody.hpp').is_file()))"
    OUTPUT_VARIABLE PIN_PREFIX OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)
endif()
list(PREPEND CMAKE_PREFIX_PATH "${PIN_PREFIX}")
find_package(pinocchio 4.1.0 EXACT REQUIRED CONFIG)
if(NOT EXISTS "${PIN_PREFIX}/include/eiquadprog/eiquadprog-rt.hpp")
  message(FATAL_ERROR "Install eiquadprog 1.3.2 in the Pinocchio prefix")
endif()
pybind11_add_module(_native NO_EXTRAS cpp/src/solver.cpp cpp/src/bindings.cpp)
target_compile_features(_native PRIVATE cxx_std_17)
target_include_directories(_native PRIVATE cpp/include "${PIN_PREFIX}/include")
target_link_libraries(_native PRIVATE pinocchio::pinocchio_default pinocchio::pinocchio_parsers)
set_target_properties(_native PROPERTIES
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH "$ORIGIN/../cmeel.prefix/lib"
  INSTALL_RPATH_USE_LINK_PATH FALSE)
install(TARGETS _native LIBRARY DESTINATION fairino_ik)

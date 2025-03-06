cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
  "components/"
  "components/espp/components/"
)

set(
  COMPONENTS
  "main esptool_py esp-box filters timer fluidsim"
  CACHE STRING
  "List of components to include"
  )

project(esp-box-fluidsim)

set(CMAKE_CXX_STANDARD 23)

project(tutorial-display-3D)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_ustk_gui)

include_directories(${VTK_INCLUDE_DIRS})
#message(${VTK_INCLUDE_DIRS})

set(tutorial_cpp
  display3D.cpp
  display2D.cpp
  )

#message(${VTK_LIBRARIES})

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorial")
  endif()
endforeach()


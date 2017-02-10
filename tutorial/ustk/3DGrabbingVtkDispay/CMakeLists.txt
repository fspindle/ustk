project(tutorial-grabbing-display-3D)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_ustk_gui visp_ustk_grabber)

set(tutorial_grabbing_display_3D_cpp
  tutorial-ultrasonix-grabber-2D-display.cpp)

foreach(cpp ${tutorial_grabbing_display_3D_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()


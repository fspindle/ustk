vp_add_module(ustk_gui visp_core visp_io visp_gui visp_ustk_core visp_ustk_io)

if(NOT VTK_FOUND)
  VP_OPTION(USE_VTK       VTK        ""    "Include vtk support"          "" ON)
endif()

find_package(VTK REQUIRED)

if(VTK_FOUND AND USE_VTK)
  # Add preprocessor definitions needed to use VTK.
  set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS ${VTK_DEFINITIONS})

  # Add include directories needed to use VTK.
  include_directories(${VTK_INCLUDE_DIRS})

  # Add link directories needed to use VTK.
  link_directories(${VTK_LIBRARY_DIRS})
else()
  message(WARNING "VTK not found for ustk gui module")
endif()

#Find Qt
#if(NOT QT_FOUND)
#  VP_OPTION(USE_QT       Qt4        ""    "Include Qt4 support"          "" ON)
#endif()

#if(VTK_FOUND AND USE_VTK)
#  MESSAGE(STATUS "Found Qt4")
#  list(APPEND opt_incs ${QT_USE_FILE})
#  list(APPEND opt_libs Qt5::Widgets)
#endif()

vp_glob_module_sources()
vp_module_include_directories(${VTK_INCLUDE_DIRS})
vp_create_module(${VTK_LIBRARIES})

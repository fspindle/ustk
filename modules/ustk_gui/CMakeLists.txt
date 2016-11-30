vp_add_module(ustk_gui visp_core visp_io visp_gui visp_ustk_core visp_ustk_io)

#Find VTK
if(NOT VTK_FOUND)
  VP_OPTION(USE_VTK       VTK        ""    "Include vtk support"          "" ON)
endif()

set(opt_incs "")
set(opt_libs "")

if(VTK_FOUND AND USE_VTK)
      list(APPEND opt_incs ${VTK_USE_FILE})
      list(APPEND opt_libs ${VTK_LIBRARIES})
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
vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})

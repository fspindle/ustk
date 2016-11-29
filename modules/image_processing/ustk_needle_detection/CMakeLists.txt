vp_add_module(ustk_needle_detection visp_core visp_gui visp_ustk_core visp_ustk_io)

if(NOT VTK_FOUND)
  VP_OPTION(USE_VTK       VTK        ""    "Include vtk support"          "" ON)
endif()

set(opt_incs "")
set(opt_libs "")

if(VTK_FOUND AND USE_VTK)
      list(APPEND opt_incs ${VTK_INCLUDE_DIR})
      list(APPEND opt_libs ${VTK_LIBRARIES})
else()
  message(WARNING "VTK not found for ustk needle detection module")
endif()

vp_glob_module_sources()
vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})

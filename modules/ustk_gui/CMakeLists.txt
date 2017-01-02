if(NOT VTK_FOUND)
  #VP_OPTION(USE_VTK       VTK        ""    "Include vtk support"          "" ON IF ${VTK_VERSION} VERSION_EQUAL 6.0.0 OR ${VTK_VERSION} VERSION_GREATER 6.0.0)
  VP_OPTION(USE_VTK       VTK        ""    "Include vtk support"          "" ON)
endif()

if(VTK_FOUND AND ${VTK_VERSION} VERSION_LESS 6.0.0)
  message(WARNING "vtk-${VTK_VERSION} detected but too old to be used by ustk. You should install at least vtk-6.0.0 to use ustk_gui module.")
  set(USE_VTK OFF CACHE BOOL "Enables vtk usage" FORCE)
endif()

if(USE_VTK)
  set(USTK_HAVE_VTK_VERSION "(${VTK_VERSION_MAJOR}<<16 | ${VTK_VERSION_MINOR}<<8 | ${VTK_VERSION_PATCH})") # for vpConfig.h

  list(APPEND opt_incs ${VTK_INCLUDE_DIRS})

  # list(APPEND opt_libs ${VTK_LIBRARIES})
  # Using VTK_LIBRARIES works to build visp/ustk library, examples, demos and test thanks to the components,
  # but not tutorials when they are build outside ViSP as they are stand alone CMake projects that use
  # ViSP as a 3rd party.
  # To be clear VTK_LIBRARIES contains VTK 3rd party such as vtkalglib and not /usr/local/Cellar/vtk/6.3.0/lib/libvtkalglib-6.3.1.dylib
  # full path as requested to use ViSP as 3rd party. This is the case for all VTK libraries we depend on.
  # The build of works with VTK_LIBRARIES since in that case thanks to vtkalglib properties, CMake
  # is able to find the real name and location of the libraries.
  # But when ViSP is used as a 3rd party where it should import VTK libraries, it doesn't work with
  # VTK_LIBRARIES.
  # The solution here is to get the full location of VTK_LIBRARIES libraries thanks to the properties and link
  # with these names.
  # An other way could be to include vtkConfig.cmake, but in that case, visp-config and visp.pc
  # will not be able to give the names of VTK libraries when used without CMake.

  set(config_ "NONE" "RELEASE" "DEBUG" "RELEASEWITHDEBINFO" "RELWITHDEBINFO")
  foreach(lib_ ${VTK_LIBRARIES})
    foreach(imp_config_ ${config_})
      get_target_property(lib_property_${imp_config_}_ ${lib_} IMPORTED_LOCATION_${imp_config_})

      # Under Unix, there is no specific suffix for PCL/VTK libraries.         # Under Windows, we add the "optimized", "debug" specific keywords
      if(WIN32 AND EXISTS "${lib_property_${imp_config_}_}" AND "${imp_config_}" MATCHES "RELEASE") # also valid for RELEASEWITHDEBINFO
        list(APPEND opt_libs optimized "${component_property_${imp_config_}_}")
      elseif(WIN32 AND EXISTS "${lib_property_${imp_config_}_}" AND "${imp_config_}" MATCHES "DEBUG")
        list(APPEND opt_libs debug     "${lib_property_${imp_config_}_}")
      elseif(EXISTS "${lib_property_${imp_config_}_}")
        list(APPEND opt_libs "${lib_property_${imp_config_}_}")
      endif()
    endforeach()
  endforeach()
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

vp_add_module(ustk_gui visp_ustk_core)
vp_glob_module_sources()
vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})

VP_SET(USTK_HAVE_VTK TRUE IF (BUILD_MODULE_visp_ustk_gui AND USE_VTK))

vp_add_config_file("cmake/templates/usGuiConfig.h.in")

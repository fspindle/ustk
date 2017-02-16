if(UNIX)
  #ultrasonix grabber not available for window

    # Instruct CMake to run moc automatically when needed.
    set(CMAKE_AUTOMOC ON)

  message(STATUS "find Qt5 for ustk_grabber_volume")
  #Qt 5
find_package(Qt5 COMPONENTS Widgets REQUIRED QUIET)
    qt5_wrap_ui(UI_Srcs ${UIs})

    list(APPEND opt_incs "${Qt5Widgets_INCLUDE_DIRS}")

    # list(APPEND opt_libs "${Qt5Widgets_LIBRARIES}")
    # Using Qt5Widgets_LIBRARIES works to build visp/ustk library, examples, demos and test thanks to the components,
    # but not tutorials when they are build outside ViSP as they are stand alone CMake projects that use
    # ViSP as a 3rd party.

    foreach(lib_ ${Qt5Widgets_LIBRARIES})
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
        get_target_property(lib_deps_${imp_config_}_ ${lib_} IMPORTED_LINK_INTERFACE_LIBRARIES_${imp_config_})

        if(lib_deps_${imp_config_}_)
          foreach(deps_ ${lib_deps_${imp_config_}_})
            get_target_property(deps_location_ ${deps_} IMPORTED_LOCATION_${imp_config_})
            if(EXISTS "${deps_location_}")
              if(WIN32 AND "${imp_config_}" MATCHES "RELEASE")
                list(APPEND opt_libs optimized ${deps_location_})
              elseif(WIN32 AND "${imp_config_}" MATCHES "DEBUG")
                list(APPEND opt_libs debug ${deps_location_})
              else()
                list(APPEND opt_libs ${deps_location_})
              endif()
            endif()
          endforeach()
        endif()

      endforeach()
    endforeach()

  #Qt 4
 if(false)
  #if(VTK_QT_VERSION VERSION_EQUAL "4")

    #Qt linking
    find_package(Qt4 REQUIRED QtGui QtCore)
    # Use what VTK built with
    set(QT_QMAKE_EXECUTABLE ${VTK_QT_QMAKE_EXECUTABLE} CACHE FILEPATH "")
    set(QT_MOC_EXECUTABLE ${VTK_QT_MOC_EXECUTABLE} CACHE FILEPATH "")
    set(QT_UIC_EXECUTABLE ${VTK_QT_UIC_EXECUTABLE} CACHE FILEPATH "")

    list(APPEND opt_incs "${QT_QTGUI_INCLUDE_DIR}")
    #list(APPEND opt_incs "${QT_INCLUDE_DIR}/QtCore") # seems not useful
    list(APPEND opt_libs ${QT_QTGUI_LIBRARIES})
    list(APPEND opt_libs ${QT_QTCORE_LIBRARIES})
  endif()

  message(STATUS "adding module ustk_grabber_volume")
    vp_add_module(ustk_grabber_volume visp_ustk_grabber visp_ustk_gui)
    vp_glob_module_sources()
    vp_module_include_directories(${opt_incs})
    vp_create_module(${opt_libs})

    vp_add_tests()

endif()

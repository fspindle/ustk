if(UNIX)
  #ultrasonix grabber not available for window

  #if(VTK_QT_VERSION VERSION_EQUAL "4")
    # Instruct CMake to run moc automatically when needed.
    set(CMAKE_AUTOMOC ON)

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


    vp_add_module(ustk_grabber_volume visp_ustk_grabber visp_ustk_gui)
    vp_glob_module_sources()
    vp_module_include_directories(${opt_incs})
    vp_create_module(${opt_libs})

    vp_add_tests()
  #endif()
endif()

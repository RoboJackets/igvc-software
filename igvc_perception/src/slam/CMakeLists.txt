# qt components used (also by qglviewer): Core Gui Xml OpenGL Widgets

include_directories(${CSPARSE_INCLUDE_DIR}
    ${QGLVIEWER_INCLUDE_DIR}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Xml_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5OpenGL_INCLUDE_DIRS}
    ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}
    )

QT5_WRAP_UI(UI_HEADERS base_main_window.ui)
QT5_WRAP_CPP(UI_SOURCES main_window.h)

add_executable(slam main.cpp)

set_target_properties(slam PROPERTIES OUTPUT_NAME slam${EXE_POSTFIX})

if(Qt5_POSITION_INDEPENDENT_CODE)
    set_property(TARGET slam PROPERTY COMPILE_FLAGS -fPIC)
    message(STATUS "Generating position indpendent code for slam because Qt5 was built with -reduce-relocations")
    # Note: using
    #       set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    #       does not seem to work: This generates some libraries with -fPIE which is not enough for Qt...
endif()


target_link_libraries(slam 
	${catkin_LIBRARIES} 
	${PCL_LIBRARIES}
	${QGLVIEWER_LIBRARY}
	${Qt5Core_LIBRARIES}
	${Qt5Gui_LIBRARIES}
	${Qt5Xml_LIBRARIES}
	${Qt5Widgets_LIBRARIES}
	${Qt5OpenGL_LIBRARIES}
	${OPENGL_gl_LIBRARY} 
	${OPENGL_glu_LIBRARY}
)

install(
    TARGETS slam
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

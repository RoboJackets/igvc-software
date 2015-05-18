cmake_minimum_required(VERSION 2.8.11)

# Find the QtWidgets library
find_package(Qt5Core 5.2 REQUIRED)
find_package(Qt5Gui 5.2 REQUIRED)
find_package(Qt5Widgets 5.2 REQUIRED)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

QT5_WRAP_UI(UIS_HDRS
    mainwindow.ui
)

add_executable(dashboard main.cpp mainwindow.cpp mainwindow.ui speedometer.cpp qnode.cpp mapwidget.cpp)
target_link_libraries(dashboard ${catkin_LIBRARIES} Qt5::Widgets Qt5::Core Qt5::Gui)

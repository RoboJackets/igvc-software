# How To Make an rviz Panel

All rviz panels are classes that inherit from rviz::Panel. Any catkin package can export a class library defining its own set of Panel subclasses, allowing packages to create their own custom rviz panels.

To create your own panel, follow these general steps:

1. Create a new subclass of rviz::Panel in the `igvc_rviz_plugins` namespace.
2. Implement your panel and add the `PLUGINLIB_EXPORT_CLASS(...)` macro to the end of your *.cpp file.
3. Add your *.h and *.cpp files to `rviz_plugins_HDRS` and `rviz_plugins_SRCS` in *CMakeLists.txt*.
4. Add a new <class> tag to *igvc_rviz_plugins*'s *plugin_description.xml* file.

That's it! At this point, you should be able to build the package with `catkin_make` in your catkin workspace. Your new panel will show up in rviz when you click *Panels->Add New Panel*.

To see more details about how to implement an rviz panel, check out *ExamplePanel.h* and *ExamplePanel.cpp*.
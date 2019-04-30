#include <igvc_rviz_plugins_old/example_panel.h>
#include <pluginlib/class_list_macros.h>
#include <QVBoxLayout>

/*
 * Just as in the header, everything needs to happen in the igvc_rviz_plugins_old namespace.
 */
namespace igvc_rviz_plugins
{
ExamplePanel::ExamplePanel(QWidget *parent) : rviz::Panel(parent)  // Base class constructor
{
  // Panels are allowed to interact with NodeHandles directly just like ROS nodes.
  ros::NodeHandle handle;

  // Initialize a label for displaying some data
  QLabel *label = new QLabel("0 m/s");

  /* Initialize our subscriber to listen to the /speed topic.
   * Note the use of boost::bind, which allows us to give the callback a pointer to our UI label.
   */
  speed_subscriber = handle.subscribe<igvc_msgs::velocity_pair>(
      "/encoders", 1, boost::bind(&ExamplePanel::speed_callback, this, _1, label));

  /* Use QT layouts to add widgets to the panel.
   * Here, we're using a VBox layout, which allows us to stack all of our widgets vertically.
   */
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(label);
  setLayout(layout);
}

void ExamplePanel::speed_callback(const igvc_msgs::velocity_pairConstPtr &msg, QLabel *label)
{
  // Create the new contents of the label based on the speed message.
  auto text = "Left: " + std::to_string(msg->left_velocity) + " m/s\n" +
              "Right: " + std::to_string(msg->right_velocity) + " m/s";
  // Set the contents of the label.
  label->setText(text.c_str());
}

// void ExamplePanel::paintEvent(QPaintEvent *event)  {
//
//}
}  // namespace igvc_rviz_plugins_old

/*
 * IMPORTANT! This macro must be filled out correctly for your panel class.
 */
PLUGINLIB_EXPORT_CLASS(igvc_rviz_plugins::ExamplePanel, rviz::Panel)

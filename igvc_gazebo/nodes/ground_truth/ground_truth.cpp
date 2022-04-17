#include "ground_truth.h"

ground_truth::ground_truth(): pNh("~") {

  //initialize parameters 

	// assertions::getParam(pNh, "pose",  pose);
	// assertions::getParam(pNh, "last_estimate", last_estimate);
	assertions::param(pNh, "ground_truth_sub_topic", sub_topic, std::string("/ground_truth/state_raw"));
	assertions::param(pNh, "ground_truth_pub_topic", pub_topic, std::string("/ground_truth"));
	assertions::param(pNh, "longitude", longitude, -84.405001);
	assertions::param(pNh, "latitude", latitude, 33.774497);
  
	
  // initialize publishers and subscribers 
  ground_truth_pub = nh.advertise<nav_msgs::Odometry>(pub_topic, 1);
  ground_truth_sub = nh.subscribe(sub_topic, 10, &ground_truth::groundTruthCallback, this);
  estimate_sub = nh.subscribe("/odometry/filtered", 1, &ground_truth::odomCallback, this);

  RobotLocalization::NavsatConversions::UTM(latitude, longitude, &utm_x, &utm_y);
  utm_to_odom.setOrigin(
    tf::Vector3(utm_x - og_pose.pose.pose.position.x, utm_y - og_pose.pose.pose.position.y, 0.0));
  utm_to_odom.setRotation(tf::createQuaternionFromYaw(M_PI));
  utm_timer = nh.createTimer(ros::Duration(1.0), boost::bind(&ground_truth::utm_callback, this, _1, utm_to_odom.inverse()));
}


void ground_truth:: odomCallback(const nav_msgs::Odometry::ConstPtr msg){
  last_estimate = msg->header.stamp;
}

void ground_truth::groundTruthCallback(const nav_msgs::Odometry::ConstPtr msg){
	// get the starting location as the origin
  if  (msg->header.stamp.toSec() == 0)
  {
    og_pose.pose = msg->pose;
    og_pose.header = msg->header;
    og_pose.pose.pose.position.x = msg->pose.pose.position.x;
    og_pose.pose.pose.position.y = msg->pose.pose.position.y;
    ROS_INFO_STREAM("setting og_pose to " << og_pose.pose.pose.position.x << ", "
                                            << og_pose.pose.pose.position.y);

    nav_msgs::Odometry initial;
    initial.twist = msg->twist;
    initial.header = msg->header;
    initial.child_frame_id = "base_footprint";
    initial.header.frame_id = "odom";

    ground_truth_pub.publish(initial);
    //publishes a (0, 0) message to /ground_truth_state_raw
  }
  else
  {
    nav_msgs::Odometry result;
    result.pose = msg->pose;

    // use the initial location as an offset (makes the starting location 0, 0)
    result.pose.pose.position.x = msg->pose.pose.position.x - og_pose.pose.pose.position.x;
    result.pose.pose.position.y = msg->pose.pose.position.y - og_pose.pose.pose.position.y;

    result.twist = msg->twist;

    // set up the correct header
    result.header = msg->header;
    result.child_frame_id = "base_footprint";
    result.header.frame_id = "odom";

    //result done

    tf::Quaternion quat;
    tf::Vector3 pos;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::pointMsgToTF(result.pose.pose.position, pos);

    // publish odom message
    ground_truth_pub.publish(result);

    // publish transform for tf if there has not been a update from the localization node in the last second
    // since it also publishes the same transform
    if (std::abs(msg->header.stamp.toSec() - last_estimate.toSec()) > 1.0)
    {
      tf::Transform transform{ quat, pos };
      br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "odom", "base_footprint"));
    }
  }
}
void ground_truth::utm_callback(const ros::TimerEvent& event, const tf::Transform& odom_to_utm)
{
  tf::StampedTransform transform;
  utm_enabled = true;

  if (utm_enabled)
  {
    bool found = true;
    try
    {
      tf_listener.lookupTransform("odom", "utm", ros::Time(0), transform);
    }
    catch (const tf::TransformException& ex)
    {
      found = false;
    }

    if (found && transform.getRotation() != odom_to_utm.getRotation() &&
        transform.getOrigin() != odom_to_utm.getOrigin())
    {
      ROS_WARN_STREAM("Anther odom -> utm tf broadcast detected. Disabling ground_truth odom -> utm tf broadcast.");
      utm_enabled = false;
      return;
    }
    br.sendTransform(tf::StampedTransform(odom_to_utm, event.current_real, "odom", "utm"));
  }
}


int main(int argc, char **argv){
	ros::init(argc, argv, "ground_truth");
	ground_truth ground_truth_obj;
	ros::spin();
}



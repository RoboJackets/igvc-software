#!/usr/bin/env python

import rospy
from visualizer import Visualizer

if __name__ == '__main__':
    rospy.init_node('nn_viz')

    # Read ros params.
    input_topic = rospy.get_param('~input_topic')
    output_topic = rospy.get_param('~output_topic')
    image_resize_width = rospy.get_param('~image_resize_width')
    image_resize_height = rospy.get_param('~image_resize_height')
    model_path = rospy.get_param('~model_path')
    force_cpu = rospy.get_param('~force_cpu')

    rospy.loginfo("[nn_viz] Testing the model %s", model_path)

    Visualizer(input_topic, output_topic, image_resize_width, image_resize_height, model_path, force_cpu)
    rospy.loginfo("[nn_viz] NN visualizer up!")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr("Shutting down")

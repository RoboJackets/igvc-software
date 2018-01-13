#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import scipy
import scipy.misc
import numpy as np
import tensorflow as tf


class CNN:

    def __init__(self, subscriber_topic, publisher_topic, model_filename, resize_width,
            resize_height, force_cpu):
        self.graph = self.load_graph(model_filename)
        config = tf.ConfigProto(
                        device_count = {'GPU': 0}
        )
        if force_cpu:
            self.sess = tf.Session(config=config,graph=self.graph)
        else:
            self.sess = tf.Session(graph=self.graph)

        self.resize_width = resize_width
        self.resize_height = resize_height

        self.publisher = rospy.Publisher(publisher_topic, Image)
        self.subscriber = rospy.Subscriber(subscriber_topic, Image, self.image_callback, queue_size=1)
        self.bridge = CvBridge()
        


    def load_graph(self, frozen_graph_filename):
        # We load the protobuf file from the disk and parse it to retrieve the 
        # unserialized graph_def
        with tf.gfile.GFile(frozen_graph_filename, "rb") as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())

        # Then, we import the graph_def into a new Graph and returns it 
        with tf.Graph().as_default() as graph:
            # The name var will prefix every op/nodes in your graph
            # Since we load everything in a new graph, this is not needed
            tf.import_graph_def(graph_def, name='')
        return graph



    def image_callback(self, image_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        cv_image = cv2.resize(cv_image, (self.resize_width, self.resize_height))
        im = np.asarray(cv_image)

        image_pl = self.graph.get_tensor_by_name('Placeholder_1:0')
        softmax = self.graph.get_tensor_by_name('Validation/decoder/Softmax:0')

        output = self.sess.run([softmax], feed_dict={image_pl: im})
        shape = im.shape
        output_image = output[0][:, 1].reshape(shape[0], shape[1])

        threshold = 0.5
        im_threshold = output_image > threshold
        im_threshold = np.uint8(255*im_threshold)
        cv_output = cv2.cvtColor(im_threshold, cv2.COLOR_GRAY2BGR)
        self.publisher.publish(self.bridge.cv2_to_imgmsg(cv_output, 'bgr8'))




if __name__ == '__main__':
    rospy.init_node('cnn')
    image_topic = rospy.get_param('/image_topic')
    image_resize_width = rospy.get_param('/image_resize_width')
    image_resize_height = rospy.get_param('/image_resize_height')
    model_path = rospy.get_param('/model_path')
    force_cpu = rospy.get_param('/force_cpu')
    CNN(image_topic, '/semantic_segmentation', model_path, image_resize_width,
            image_resize_height, force_cpu)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

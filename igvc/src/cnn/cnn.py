#!/usr/bin/env python
import rospy
import cv2
from image_geometry import PinholeCameraModel
import tf as ros_tf
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge, CvBridgeError
import scipy
import scipy.misc
import numpy as np
import tensorflow as tf
from timeit import default_timer as timer


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
        self.bridge = CvBridge()

	try:
            camera_info = rospy.wait_for_message('/usb_cam_center/camera_info', CameraInfo, timeout=5)
        except(rospy.ROSException), e:
            print "Camera info topic not available"
            print e
            exit()

        # width and height adjust factors
        waf = float(resize_width) / camera_info.width
        haf = float(resize_height) / camera_info.height
        camera_info.height = resize_height
        camera_info.width = resize_width

        # adjust the camera matrix
        K = camera_info.K
        camera_info.K = (K[0]*waf,         0.,  K[2]*waf,
                                0.,  K[4]*haf,  K[5]*haf,
                                0.,        0.,         1.)

        # adjust the projection matrix
        P = camera_info.P
        camera_info.P = (P[0]*waf,        0.,  P[2]*waf,  0.,
                               0.,  P[5]*haf,  P[6]*haf,  0.,
                               0.,        0.,        1.,  0.)

        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(camera_info)
        print camera_info

        transform_listener = ros_tf.TransformListener()
        transform_listener.waitForTransform('/base_footprint', '/optical_cam_center', rospy.Time(0), rospy.Duration(5.0))
        cam_transform_translation, cam_transform_rotation = transform_listener.lookupTransform('/base_footprint', '/optical_cam_center', rospy.Time(0))
        self.cam_transform_rotation_matrix = ros_tf.transformations.quaternion_matrix(cam_transform_rotation)[:-1,:-1]
        self.cam_transform_translation = np.asarray(cam_transform_translation)
        print self.cam_transform_translation
        print self.cam_transform_rotation_matrix

        self.init_point_cloud_array()

        self.im_publisher = rospy.Publisher(publisher_topic, Image, queue_size=1)
        self.cloud_publisher = rospy.Publisher("/semantic_segmentation_cloud", PointCloud, queue_size=1)
        self.subscriber = rospy.Subscriber(subscriber_topic, Image, self.image_callback, queue_size=1, buff_size=10**8)
        




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



    def init_point_cloud_array(self):
        world_points_array = np.empty( (self.resize_height,self.resize_width) ,dtype=object)
        for r,c in np.ndindex(self.resize_height,self.resize_width):
            ray = np.asarray( self.camera_model.projectPixelTo3dRay( (c,r) ) )
            ray_in_world = np.matmul( self.cam_transform_rotation_matrix, ray )
            scale = -self.cam_transform_translation[2] / ray_in_world[2]
            world_point = scale * ray_in_world + self.cam_transform_translation
            world_points_array[r,c] = Point32( world_point[0], world_point[1], world_point[2] )
        self.world_point_array = world_points_array



    def image_to_point_cloud(self, input_image):
        half_rows = np.size(input_image, axis=0) / float(2)
        world_points = []
        for r,c in zip(*input_image.nonzero()):
            #if r < half_rows:
            #    continue
            ray = np.asarray( self.camera_model.projectPixelTo3dRay( (c,r) ) )
            ray_in_world = np.matmul( self.cam_transform_rotation_matrix, ray )
            scale = -self.cam_transform_translation[2] / ray_in_world[2]
            world_point = scale * ray_in_world + self.cam_transform_translation
            world_points.append( Point32( world_point[0], world_point[1], world_point[2] ) )

        return world_points



    def image_callback(self, image_msg):
        start = timer()
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
        world_points = self.world_point_array[im_threshold].tolist()

        im_threshold = np.uint8(255*im_threshold)
        cv_output = cv2.cvtColor(im_threshold, cv2.COLOR_GRAY2BGR)
        msg_out = self.bridge.cv2_to_imgmsg(cv_output, 'bgr8')
        msg_out.header.stamp = image_msg.header.stamp
        self.im_publisher.publish(msg_out)

        cloud_msg = PointCloud()
        cloud_msg.header.stamp = image_msg.header.stamp
        cloud_msg.header.frame_id = 'base_footprint'
        cloud_msg.points = world_points
        self.cloud_publisher.publish(cloud_msg)
        print timer() - start





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

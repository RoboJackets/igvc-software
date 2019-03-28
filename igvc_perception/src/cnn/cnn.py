#!/usr/bin/env python
import cv2
import functools
import message_filters
import numpy as np
import pdb
from PIL import Image
import sys
from timeit import default_timer as timer
import torch
from torch.autograd import Variable
from torchvision import transforms

sys.path.append('../train_eval')
from train_eval.models.model import UNet

# ROS imports.
import rospy
import tf as ros_tf

from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image as ImMsg
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

transform = transforms.Compose([
                 transforms.ToTensor(),
            ])


class CNN():

    def __init__(self, camera_names, publisher_topic, model_filename, resize_width,
            resize_height, force_cpu, line_thresh):

        # Load the model.
        self.graph = UNet([3,resize_width,resize_height], 3)
        self.graph.load_state_dict(torch.load(model_filename))
        
        self.force_cpu = force_cpu

        if not self.force_cpu:
            self.graph.cuda()
        self.graph.eval()

        self.bridge = CvBridge()

        self.resize_width = resize_width
        self.resize_height = resize_height

        self.line_thresh = line_thresh
        
        # Setup camera model and subsciber for each camera.
        self.camera_models = {}
        self.cam_transform_rotation_matrices = {}
        self.cam_transform_translations = {}
        self.world_point_arrays = {}
        self.subscribers = []
        self.im_publishers = {}
        self.cloud_publishers = {}

        for camera_name in camera_names:
            rospy.loginfo('Setting up %s.' % camera_name)

            try:
                camera_info = rospy.wait_for_message('/%s/camera_info' % camera_name, CameraInfo, timeout=5)
            except(rospy.ROSException), e:
                rospy.logerr('Camera info for %s not available.' % camera_name)
                exit()

            # width and height adjust factors.
            waf = float(resize_width) / camera_info.width
            haf = float(resize_height) / camera_info.height
            camera_info.height = resize_height
            camera_info.width = resize_width

            # adjust the camera matrix.
            K = camera_info.K
            camera_info.K = (K[0]*waf,         0.,  K[2]*waf,
                                    0.,  K[4]*haf,  K[5]*haf,
                                    0.,        0.,         1.)

            # adjust the projection matrix.
            P = camera_info.P
            camera_info.P = (P[0]*waf,        0.,  P[2]*waf,  0.,
                                   0.,  P[5]*haf,  P[6]*haf,  0.,
                                   0.,        0.,        1.,  0.)

            self.camera_models[camera_name] = PinholeCameraModel()
            self.camera_models[camera_name].fromCameraInfo(camera_info)
            rospy.loginfo(camera_info)

            # Get transform between camera and base_footprint.
            transform_listener = ros_tf.TransformListener()

            cam_frame_name = camera_name.replace('usb_cam', '/optical_cam')
            transform_listener.waitForTransform('/base_footprint', cam_frame_name, rospy.Time(0), rospy.Duration(5.0))
            cam_transform_translation, cam_transform_rotation = transform_listener.lookupTransform('/base_footprint', cam_frame_name, rospy.Time(0))
            self.cam_transform_rotation_matrices[camera_name] = ros_tf.transformations.quaternion_matrix(cam_transform_rotation)[:-1,:-1]
            self.cam_transform_translations[camera_name] = np.asarray(cam_transform_translation)
            print self.cam_transform_translations[camera_name]
            print self.cam_transform_rotation_matrices[camera_name]

            # Create a mapping between image pixels and world coordinates (flat plane assumption).
            self.init_point_cloud_array(camera_name)

            # Create image and pointcloud publishers.
            self.im_publishers[camera_name] = rospy.Publisher('%s/%s' % (publisher_topic, camera_name), ImMsg, queue_size=1)
            self.cloud_publishers[camera_name] = rospy.Publisher('%s/%s' % ("/semantic_segmentation_cloud", camera_name), PointCloud2, queue_size=1)

            print('Finished setting up %s.' % camera_name)

        

        # Initialize subscibers at the end so that callbacks are not active until this stage.
        for camera_name in camera_names:
            # Use the same callback for every camera.
            self.subscribers.append(rospy.Subscriber('/%s/image_raw/compressed' % camera_name, CompressedImage, functools.partial(self.image_cb, camera_name), queue_size=1, buff_size=10**8))
        rospy.loginfo('Line detector is running.')

    # This function projects image pixels into world coordinates using a flat plane assumption.
    def init_point_cloud_array(self, camera_name):
        camera_model = self.camera_models[camera_name]
        cam_transform_translation = self.cam_transform_translations[camera_name]
        cam_transform_rotation_matrix = self.cam_transform_rotation_matrices[camera_name]

        world_points_array = np.empty( (self.resize_height,self.resize_width,3), dtype=np.float32)
        for r,c in np.ndindex(self.resize_height,self.resize_width):
            ray = np.asarray(camera_model.projectPixelTo3dRay((c,r)))
            ray_in_world = np.matmul(cam_transform_rotation_matrix, ray)
            scale = -cam_transform_translation[2] / ray_in_world[2]
            world_point = scale * ray_in_world + cam_transform_translation
            world_points_array[r,c] = [world_point[0], world_point[1], world_point[2]]
        self.world_point_arrays[camera_name] = world_points_array

    def image_cb(self, camera_name, data):
        # Track inference time.
        start = timer()

        # Resize image and convert to tensor.
        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image_np = cv2.resize(image_np, dsize=(self.resize_width,self.resize_height))
        img_to_tensor = Image.fromarray(image_np)
        img_tensor = transform(img_to_tensor)

        if not self.force_cpu:
            img_tensor = Variable(img_tensor.unsqueeze(0)).cuda()
        else:
            img_tensor = Variable(img_tensor.unsqueeze(0))

        # Inference.
        output = self.graph(img_tensor)
        output_data = output.cpu().data.numpy()[0][0]

        # Network output values above threshold are lines.
        im_threshold = output_data > self.line_thresh

        # Get world coordinates of detected lines.
        world_points = self.world_point_arrays[camera_name][im_threshold]

        # Publish segmentation map.
        im_threshold = np.uint8(255*im_threshold)
        cv_output = cv2.cvtColor(im_threshold, cv2.COLOR_GRAY2BGR)
        msg_out = self.bridge.cv2_to_imgmsg(cv_output, 'bgr8')
        msg_out.header.stamp = data.header.stamp
        self.im_publishers[camera_name].publish(msg_out)

        # Publish pointcloud.
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = data.header.stamp
        cloud_msg.header.frame_id = 'base_footprint'
        cloud_msg.height = 1
        cloud_msg.width = len(world_points)
        cloud_msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12
        cloud_msg.row_step = 3 * len(world_points)
        cloud_msg.data = world_points.tostring()

        self.cloud_publishers[camera_name].publish(cloud_msg)

        end = timer()
        #print end - start

if __name__ == '__main__':
    rospy.init_node('cnn')

    # Read ros params.
    camera_names = rospy.get_param('camera_names')
    image_resize_width = rospy.get_param('image_resize_width')
    image_resize_height = rospy.get_param('image_resize_height')
    model_path = rospy.get_param('model_path')
    force_cpu = rospy.get_param('force_cpu')
    line_thresh = rospy.get_param('line_thresh')

    CNN(camera_names, '/semantic_segmentation', model_path, image_resize_width,
        image_resize_height, force_cpu, line_thresh)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr("Shutting down")

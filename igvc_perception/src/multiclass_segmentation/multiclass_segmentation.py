#!/usr/bin/env python
from sensor_msgs.msg import Image as ImMsg
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge, CvBridgeError
import tf as ros_tf
import rospy
# from train_eval.models.model import UNet
import cv2
import functools
import numpy as np
import sys
from timeit import default_timer as timer
import torch
from torch.autograd import Variable
import segmentation_models_pytorch as smp

import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

# ROS imports.


class SegmentationModel(object):
    """
    Segmentation and Pinhole projection for line detection
    """

    def __init__(
            self,
            camera_names,
            publisher_topic,
            resize_width,
            resize_height,
            use_preexisting_segmentation,
            **kwargs):

        self.use_preexisting_segmentation = use_preexisting_segmentation
        self.resize_width = resize_width
        self.resize_height = resize_height

        self.bridge = CvBridge()

        if self.use_preexisting_segmentation:
            segmented_image_topics = kwargs["segmented_image_topics"]
        else:
            # load the model
            self.graph = smp.Unet(
                encoder_name="efficientnet-b3",
                encoder_weights="imagenet",
                classes=3,
                activation=None
            )
            # Load weights from file
            self.graph.load_state_dict(torch.load(
                kwargs["model_filename"], map_location=torch.device('cpu')
            ))

            self.force_cpu = kwargs["force_cpu"]
            self.line_thresh = kwargs["line_thresh"]

            if not self.force_cpu:
                self.graph.cuda()

            self.graph.eval()

        # Setup camera model and subscriber for each camera.
        self.camera_models = {}
        self.cam_transform_rotation_matrices = {}
        self.cam_transform_translations = {}
        self.world_point_arrays = {}
        self.subscribers = []
        self.im_publishers = {}

        for camera_name in camera_names:
            rospy.loginfo('Setting up %s.' % camera_name)

            try:
                camera_info = rospy.wait_for_message(
                    '/%s/camera_info' %
                    camera_name, CameraInfo, timeout=5)
            except rospy.ROSException:
                rospy.logerr('Camera info for %s not available.' % camera_name)
                sys.exit()
            rospy.loginfo(camera_info)

            self.camera_models[camera_name] = PinholeCameraModel()
            self.camera_models[camera_name].fromCameraInfo(camera_info)
            # # Get transform between camera and base_footprint.
            # transform_listener = ros_tf.TransformListener()
            # rospy.loginfo(ros_tf.Transformer().allFramesAsString())
            # print(camera_name)
            # cam_frame_name = "cam/" + camera_name.replace("usb_cam_", "")
            # # cam_frame_name = camera_name.replace("/raw","") + "_optical"
            # rospy.loginfo(cam_frame_name)
            # transform_listener.waitForTransform(
            #     '/base_footprint',
            #     cam_frame_name,
            #     rospy.Time(0),
            #     rospy.Duration(5.0))
            # cam_transform_translation, cam_transform_rotation = \
            #     transform_listener.lookupTransform(
            #         '/base_footprint', cam_frame_name, rospy.Time(0))
            # self.cam_transform_rotation_matrices[camera_name] = \
            #     ros_tf.transformations.quaternion_matrix(
            #         cam_transform_rotation)[:-1, :-1]
            # self.cam_transform_translations[camera_name] = np.asarray(
            #     cam_transform_translation)

            # Create image publishers.
            self.im_publishers[camera_name] = rospy.Publisher(
                '%s/%s' % (publisher_topic, camera_name), ImMsg, queue_size=1)

            print('Finished setting up %s.' % camera_name)

        if self.use_preexisting_segmentation:
            for camera_name, segmented_image_topic in zip(
                    camera_names, segmented_image_topics):
                # Use the same callback for every camera.
                self.subscribers.append(
                    rospy.Subscriber(
                        segmented_image_topic,
                        ImMsg,
                        functools.partial(
                            self.segmented_image_cb,
                            camera_name),
                        queue_size=1,
                        buff_size=10**8))
        else:
            for camera_name in camera_names:
                # Use the same callback for every camera.
                self.subscribers.append(
                    rospy.Subscriber(
                        '/%s/image_raw/compressed' %
                        camera_name,
                        CompressedImage,
                        functools.partial(
                            self.image_cb,
                            camera_name),
                        queue_size=1,
                        buff_size=10**8))

        rospy.loginfo('Line detector is running.')

    def image_cb(self, camera_name, data):
        # Track inference time.
        start = timer()

        # Convert Buffer to Image
        np_arr = np.frombuffer(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # image_np = cv2.resize(image_np, dsize=(
        # self.resize_width,self.resize_height))
        # Swap dimensions around to the dimensions the model expects
        image_np = np.swapaxes(image_np, 2, 0)
        image_np = np.swapaxes(image_np, 2, 1)
        # Convert to tensor

        img_to_tensor = torch.from_numpy(image_np).float()

        if not self.force_cpu:
            img_to_tensor = Variable(img_to_tensor.unsqueeze(0)).cuda()
        else:
            img_to_tensor = Variable(img_to_tensor.unsqueeze(0))
        output = self.graph(img_to_tensor)
        pred_mask = output.cpu().data.numpy()[0]

        # Convert back to image dimensions
        pred_mask = np.swapaxes(pred_mask, 2, 0)
        pred_mask = np.swapaxes(pred_mask, 1, 0)
        pred_mask = np.argmax(pred_mask, axis=2)

        # Network output values above threshold are lines.

        colorImg = cv2.cvtColor(pred_mask.astype(np.uint8), cv2.COLOR_GRAY2BGR)
        lineMask = np.all(colorImg == [2, 2, 2], axis=2)
        colorImg[lineMask] = [255, 0, 0]
        barrelMask = np.all(colorImg == [1, 1, 1], axis=2)
        colorImg[barrelMask] = [0, 255, 0]
        msg_out = self.bridge.cv2_to_imgmsg(colorImg, 'bgr8')
        msg_out.header.stamp = data.header.stamp
        self.im_publishers[camera_name].publish(msg_out)

        end = timer()
        # print end - start

    def segmented_image_cb(self, camera_name, data):
        # Track inference time.
        start = timer()

        # Resize image and convert to tensor.
        np_arr = self.bridge.imgmsg_to_cv2(data, "mono8")
        image_np = cv2.resize(
            np_arr,
            dsize=(
                self.resize_width,
                self.resize_height))

        # Get world coordinates of detected lines.
        im_threshold = np.array(np.rint(image_np / 255.0), dtype=np.bool_)
        world_points = self.world_point_arrays[camera_name][im_threshold]

        # Publish segmentation map.
        im_threshold = np.uint8(255 * im_threshold)
        cv_output = cv2.cvtColor(im_threshold, cv2.COLOR_GRAY2BGR)
        msg_out = self.bridge.cv2_to_imgmsg(cv_output, 'bgr8')
        msg_out.header.stamp = data.header.stamp
        self.im_publishers[camera_name].publish(msg_out)

        end = timer()
        # print end - start


if __name__ == '__main__':
    rospy.init_node('multiclass_segmentation')

    # Read ros params.
    use_preexisting_segmentation = rospy.get_param(
        'use_preexisting_segmentation', False)

    camera_names = rospy.get_param('camera_names')
    image_resize_width = rospy.get_param('image_resize_width')
    image_resize_height = rospy.get_param('image_resize_height')
    output_segmentation_topic = rospy.get_param('output_segmentation_topic')

    if use_preexisting_segmentation:
        segmented_image_topics = rospy.get_param('segmented_image_topics')
        SegmentationModel(camera_names,
                          output_segmentation_topic,
                          image_resize_width, image_resize_height,
                          use_preexisting_segmentation,
                          segmented_image_topics=segmented_image_topics)

    else:
        model_path = rospy.get_param('model_path')
        force_cpu = rospy.get_param('force_cpu')
        line_thresh = rospy.get_param('line_thresh')
        SegmentationModel(camera_names,
                          output_segmentation_topic,
                          image_resize_width, image_resize_height,
                          use_preexisting_segmentation,
                          model_filename=model_path,
                          force_cpu=force_cpu,
                          line_thresh=line_thresh)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr("Shutting down")
from typing import Tuple
import numpy as np
import cv2
import torch
from torch.autograd import Variable
from torchvision import transforms
import PIL.Image

import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image as ImMsg

from models import UNet, load_se_resnet50
import albumentations as albu
from albumentations.pytorch import ToTensor

USE_UNET = False


class Visualizer(object):
    def __init__(self, input_topic, output_topic, resize_width, resize_height, model_path,
                 force_cpu):  # type: (str, str, int, int, str, bool) -> None
        self.bridge = CvBridge()

        global USE_UNET
        if "IGVC" in model_path:
            USE_UNET = True

        if USE_UNET:
            self.graph = UNet([3, resize_width, resize_height], 3)
            self.graph.load_state_dict(torch.load(model_path))
        else:
            self.graph = load_se_resnet50(model_path)
        self.force_cpu = force_cpu and torch.cuda.is_available()

        self.resize_width, self.resize_height = resize_width, resize_height

        if not self.force_cpu:
            self.graph.cuda()
        self.graph.eval()
        self.to_tensor = transforms.Compose([transforms.ToTensor()])

        self.publisher = rospy.Publisher(output_topic, ImMsg, queue_size=1)
        self.raw_subscriber = rospy.Subscriber(input_topic, CompressedImage, self.image_cb, queue_size=1,
                                               buff_size=10 ** 8)

    @staticmethod
    def imagenet_normalization(image):  # type: (np.ndarray) -> torch.FloatTensor
        IMAGE_SIZE = 400
        transformations = albu.Compose([albu.Resize(IMAGE_SIZE, IMAGE_SIZE, p=1), albu.Normalize(), ToTensor()])
        sample = transformations(image=image)
        return sample["image"]

    def convert_to_tensor(self, image):
        image_np = np.fromstring(image.data, np.uint8)
        image_np = cv2.imdecode(image_np, cv2.IMREAD_COLOR)

        if USE_UNET:
            image_np = cv2.resize(image_np, dsize=(self.resize_width, self.resize_height))
            img_to_tensor = PIL.Image.fromarray(image_np)
            img_tensor = self.to_tensor(img_to_tensor)
            if not self.force_cpu:
                return Variable(img_tensor.unsqueeze(0)).cuda()
            else:
                return Variable(img_tensor.unsqueeze(0))
        else:
            # Convert from BGR to RGB prob
            image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
            res = self.imagenet_normalization(image_np)
            res = res.unsqueeze(0)
            if not self.force_cpu:
                return res.cuda()
            else:
                return res

    _IMAGENET_STD = (0.229, 0.224, 0.225)
    _IMAGENET_MEAN = (0.485, 0.456, 0.406)

    @staticmethod
    def tensor_to_ndimage(images, mean=_IMAGENET_MEAN, std=_IMAGENET_STD, dtype=np.float32):
        # type: (torch.Tensor, Tuple[float, float, float], Tuple[float, float, float], str) -> np.ndarray
        """
        Convert float image(s) with standard normalization to
        np.ndarray with [0..1] when dtype is np.float32 and [0..255]
        when dtype is `np.uint8`.

        Args:
            images: [B]xCxHxW float tensor
            mean: mean to add
            std: std to multiply
            dtype: result ndarray dtype. Only float32 and uint8 are supported.
        Returns:
            [B]xHxWxC np.ndarray of dtype
        """
        has_batch_dim = len(images.shape) == 4

        num_shape = (3, 1, 1)

        if has_batch_dim:
            num_shape = (1,) + num_shape

        mean = images.new_tensor(mean).view(*num_shape)
        std = images.new_tensor(std).view(*num_shape)

        images = images * std + mean

        images = images.clamp(0, 1).numpy()

        images = np.moveaxis(images, -3, -1)

        if dtype == np.uint8:
            images = (images * 255).round().astype(dtype)
        else:
            assert dtype == np.float32, "Only float32 and uint8 are supported"

        return images

    def image_cb(self, image):
        img_tensor = self.convert_to_tensor(image)

        # Inference
        output = self.graph(img_tensor)
        if USE_UNET:
            output_data = output.cpu().data.numpy()[0][0]
        else:
            output = output.sigmoid()
            mask = output.detach().cpu().numpy().astype("float")
            output_data = mask.squeeze(0).squeeze(0)
            # max_value = np.amax(output_data)
            # min_value = np.amin(output_data)
            # max_min = max_value - min_value
            # output_data -= min_value
            # output_data /= max_min

        # # Convert from 32fc1 (0 - 1) to 8uc1 (0 - 255)
        cv_output = np.uint8(255 * output_data)
        cv_output = cv2.applyColorMap(cv_output, cv2.COLORMAP_VIRIDIS)

        # Convert to ROS message to publish
        msg_out = self.bridge.cv2_to_imgmsg(cv_output, 'bgr8')
        msg_out.header.stamp = image.header.stamp
        self.publisher.publish(msg_out)

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

from models.unet import UNet


class Visualizer(object):
    def __init__(self, input_topic, output_topic, resize_width, resize_height, model_path, force_cpu):
        self.bridge = CvBridge()

        self.graph = UNet([3, resize_width, resize_height], 3)
        self.graph.load_state_dict(torch.load(model_path))
        self.force_cpu = force_cpu and torch.cuda.is_available()

        self.resize_width, self.resize_height = resize_width, resize_height

        if not self.force_cpu:
            self.graph.cuda()
        self.graph.eval()
        self.to_tensor = transforms.Compose([transforms.ToTensor()])

        self.publisher = rospy.Publisher(output_topic, ImMsg, queue_size=1)
        self.raw_subscriber = rospy.Subscriber(input_topic, CompressedImage, self.image_cb, queue_size=1,
                                               buff_size=10 ** 8)

    def convert_to_tensor(self, image):
        np_arr = np.fromstring(image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image_np = cv2.resize(image_np, dsize=(self.resize_width, self.resize_height))
        img_to_tensor = PIL.Image.fromarray(image_np)
        img_tensor = self.to_tensor(img_to_tensor)

        if not self.force_cpu:
            return Variable(img_tensor.unsqueeze(0)).cuda()
        else:
            return Variable(img_tensor.unsqueeze(0))

    def image_cb(self, image):
        img_tensor = self.convert_to_tensor(image)

        # Inference
        output = self.graph(img_tensor)
        output_data = output.cpu().data.numpy()[0][0]

        # # Convert from 32fc1 (0 - 1) to 8uc1 (0 - 255)
        cv_output = np.uint8(255 * output_data)
        cv_output = cv2.applyColorMap(cv_output, cv2.COLORMAP_JET)

        # Convert to ROS message to publish
        msg_out = self.bridge.cv2_to_imgmsg(cv_output, 'bgr8')
        msg_out.header.stamp = image.header.stamp
        self.publisher.publish(msg_out)

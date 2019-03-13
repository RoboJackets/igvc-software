from __future__ import division

import argparse
import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import pdb
from PIL import Image
import timeit
import torch
from torch.autograd import Variable
from torchvision import transforms

from models.model import UNet


parser = argparse.ArgumentParser(description='IGVC segmentation of lines.')
parser.add_argument('--load_model', type=str, default=None,
                help='Load model from .pt file.')
parser.add_argument('--img_dir', type=str, default=None,
                help='Directory containing images to evaluate.')
parser.add_argument('--anno_dir', type=str, default=None,
                help='Directory containing annotations corresponding to img_dir.')
parser.add_argument('--im_size', type=int, nargs=2, default=[400,400],
                help='Model input image dimensions.')
parser.add_argument('--vis', action='store_true', default=False,
                    help='Visualize model output if true.')
transform = transforms.Compose([
                 transforms.ToTensor(),
            ])
if __name__ == '__main__':
    args = parser.parse_args()
    img_dir = args.img_dir
    im_size = args.im_size
    anno_dir = args.anno_dir

    model = UNet([3,im_size[0],im_size[1]], 3)
    model.load_state_dict(torch.load(args.load_model))

    img_names = np.sort(os.listdir(img_dir))
    if anno_dir:
        anno_names = np.sort(os.listdir(anno_dir))
        assert len(img_names) == len(anno_names)

    model.cuda()
    model.eval()

    # Calculate statistics.
    inference_times = []
    avg_precisions = {}
    avg_recalls = {}
    avg_accuracy = 0.0

    for i in range(len(img_names)):
        img_name = img_names[i]
        print('Evaluating %s' % img_name)
        img = cv2.imread(os.path.join(img_dir,img_name))
        img = cv2.resize(img, (im_size[0],im_size[1]))

        # Get inference time.
        start_t = timeit.default_timer()

        img_to_tensor = Image.fromarray(img)
        img_tensor = transform(img_to_tensor)
        img_tensor = Variable(img_tensor.unsqueeze(0)).cuda()

        #img_tensor = Variable(img_tensor.unsqueeze(0))

        output = model(img_tensor)
        output_data = output.cpu().data.numpy()[0][0]

        end_t = timeit.default_timer()
        inference_times.append(end_t-start_t)

        if anno_dir:
            anno_name = anno_names[i]
            anno = cv2.imread(os.path.join(anno_dir, anno_name))
            anno[anno!=0] = 255
            if anno.shape[2] == 3:
                anno = anno[:,:,2]
                anno = cv2.resize(anno, (im_size[0],im_size[1]))

            # Calculate accuracy, where any value above 0.5 is true, below is false.
            acc = (np.sum(output_data[anno!=0] > 0.5) + np.sum(output_data[anno==0] < 0.5)) / float(im_size[0]*im_size[1])
            avg_accuracy += acc

            # Calculate precision and recall for different thresholds.
            for thresh in np.arange(0,1,0.05):
                recall = np.sum(output_data[anno!=0] > thresh) / np.sum(anno!=0)
                precision = np.sum(anno[output_data > thresh] != 0) / np.sum(output_data > thresh)

                if not thresh in avg_precisions:
                    avg_precisions[thresh] = 0.0
                    avg_recalls[thresh] = 0.0
                avg_recalls[thresh] += recall
                avg_precisions[thresh] += precision

        if args.vis:
            cv2.imshow('input: ', img_tensor.cpu().data.numpy()[0][0])
            cv2.imshow('output: ', output_data)

            if anno_dir:
                cv2.imshow('gt: ', anno)

            k = cv2.waitKey(50)

    if anno_dir:
        avg_accuracy /= len(img_names)
        recalls = []
        precisions = []
        keys = avg_recalls.keys()
        keys.sort()
        for key in keys:
            avg_recalls[key] /= len(img_names)
            avg_precisions[key] /= len(img_names)
            recalls.append(avg_recalls[key])
            precisions.append(avg_precisions[key])

        print('Average inference time: %f' % np.mean(inference_times))
        print('Accuracy: %f' % avg_accuracy)
        fig = plt.figure()
        plt.plot(precisions, recalls)
        plt.title('Precision recall curve of IGVC line segmentation.')
        plt.xlabel('Precision')
        plt.ylabel('Recall')
        plt.show()

    pdb.set_trace()


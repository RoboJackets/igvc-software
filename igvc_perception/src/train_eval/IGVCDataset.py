import cv2
import datetime
import glob
import ntpath
import numpy as np
import os
import pickle
from PIL import Image
import pdb
import random
import torch.utils.data as data


class IGVCDataset(data.Dataset):

    def __init__(self, root, im_size, split='train', transform=None, val_samples=1):
        self.root = root
        self.transform = transform
        self.split = split
        self.im_size = im_size
        self.val_samples = val_samples

        # Root contains a list of images to be used for the dataset.
        with open(root, 'r') as file:
            self.lines = file.read().splitlines()

        if self.split in ['train', 'val']:
            random.seed(4)
            random.shuffle(self.lines)

        self.get_paths()

    def __getitem__(self, index):
        """
        Args:
            index (int): Index
        Returns:
            tuple: (image, target) where target is index of the target class.
        """
        if self.split == 'train':
            img_path, target_path = self.train_data[index], self.train_labels[index]
        elif self.split == 'val':
            img_path, target_path = self.val_data[index], self.val_labels[index]
        elif self.split == 'test':
            img_path, target_path = self.test_data[index], self.test_labels[index]

        # TODO: Make annotation grayscale so we don't need this hardcoded layer.
        try:
            img = cv2.imread(img_path)
        except:
            pdb.set_trace()
        target = cv2.imread(target_path)[:,:,2]
        
        img = cv2.resize(img, (self.im_size[1],self.im_size[2]))
        target = cv2.resize(target, (self.im_size[1], self.im_size[2]))
        target[target != 0] = 255
        
        # doing this so that it is consistent with all other datasets
        # to return a PIL Image
        img = Image.fromarray(img)
        target = Image.fromarray(target)

        if self.transform is not None:
            img = self.transform(img)
            target = self.transform(target)

        return img, target

    def __len__(self):
        if self.split == 'train':
            return len(self.train_data)
        elif self.split == 'val':
            return len(self.val_data)
        elif self.split == 'test':
            return len(self.test_data)

    def get_paths(self):
        print('Identifying %s dataset.' % self.split)
        data = []
        labels = []

        # Get the corresponding label for each image.
        for line in self.lines:
            imgpath = line
            img_filename = ntpath.basename(imgpath)
            anno_filename = img_filename.replace('jpg', 'png')

            labpath = imgpath.replace('imgs', 'annos').replace(img_filename, anno_filename)

            if not os.path.exists(labpath):
                print('Could not find label for %s.' % imgpath)
                continue

            data.append(imgpath)
            labels.append(labpath)

        if self.split in ['train', 'val']:
            self.train_data = data
            self.train_labels = labels
            self.val_data = self.train_data[-self.val_samples:]
            self.val_labels = self.train_labels[-self.val_samples:]
            self.train_data = self.train_data[:-self.val_samples]
            self.train_labels = self.train_labels[:-self.val_samples]
        else:
            self.test_data = data
            self.test_labels = labels

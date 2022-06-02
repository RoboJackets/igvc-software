# Import dependencies
import cv2
import numpy as np
import torch
from torch.utils.data import Dataset


class SegmentationDataset(Dataset):
    # Define a dataset class
    def __init__(
        self,
        image_arr_path,
        mask_arr_path,
        im_size
    ) -> None:
        self.images = np.load(image_arr_path)
        self.masks = np.load(mask_arr_path)
        self.im_size = im_size

    def __len__(self) -> int:
        return len(self.images)

    def __getitem__(self, idx: int) -> dict:
        image = self.images[idx]
        image = cv2.resize(image, (self.im_size[1], self.im_size[2]))
        image = np.swapaxes(image, 2, 0)
        image = np.swapaxes(image, 2, 1)
        image = torch.from_numpy(image).float()
        result = {"image": image}

        if self.masks is not None:
            mask = self.masks[idx]
            mask = cv2.resize(mask, (self.im_size[1], self.im_size[2]))
            mask = np.swapaxes(mask, 2, 0)
            mask = np.swapaxes(mask, 2, 1)
            mask = torch.from_numpy(mask).float()
            result["mask"] = mask
        return result

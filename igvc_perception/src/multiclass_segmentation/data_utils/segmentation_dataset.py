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
        h, w, _ = self.images[0].shape
        self.resize = (im_size[0] != h or im_size[1] != w)

    def __len__(self) -> int:
        return len(self.images)

    def __getitem__(self, idx: int) -> dict:
        image = self.images[idx]
        if self.resize:
            image = cv2.resize(image, (self.im_size[0], self.im_size[1]))
        image = np.swapaxes(image, 2, 0)
        image = np.swapaxes(image, 2, 1)
        image = torch.from_numpy(image).float()
        result = {"image": image}

        if self.masks is not None:
            mask = self.masks[idx]
            if self.resize:
                mask = cv2.resize(mask, (self.im_size[0], self.im_size[1]))
                # Need to add dimension since mask is 1D
                mask = mask[..., np.newaxis]
            mask = np.swapaxes(mask, 2, 0)
            mask = np.swapaxes(mask, 2, 1)
            mask = torch.from_numpy(mask).float()
            result["mask"] = mask
        return result

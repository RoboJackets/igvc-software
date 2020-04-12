import numpy as np
import torch
from torch.utils.data import Dataset

# Defines and establishes a dataset class
# Defines and establishes a dataset class
class SegmentationDataset(Dataset):
    def __init__(self, image_arr_path, mask_arr_path,) -> None:
        self.images = np.load(image_arr_path)
        self.masks = np.load(mask_arr_path)

    def __len__(self) -> int:
        return len(self.images)

    def __getitem__(self, idx: int) -> dict:
        image = self.images[idx]
        image = np.swapaxes(image, 2, 0)
        image = np.swapaxes(image, 2, 1)
        image = torch.from_numpy(image).float()
        result = {"image": image}

        if self.masks is not None:
            mask = self.masks[idx]
            mask = np.swapaxes(mask, 2, 0)
            mask = np.swapaxes(mask, 2, 1)
            mask = torch.from_numpy(mask).float()
            result["mask"] = mask
        return result


# Test with:
#
# dset = SegmentationDataset(image_arr_path="/content/drive/My Drive/RoboJackets/Split_Data/train_images.npy", mask_arr_path="/content/drive/My Drive/RoboJackets/Split_Data/train_masks.npy")

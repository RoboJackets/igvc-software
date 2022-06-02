# Import dependencies
import numpy as np
from typing import List
from pathlib import Path
from torch.utils.data import Dataset
import torch
import collections
from sklearn.model_selection import train_test_split
from torch.utils.data import DataLoader
from data_utils.segmentation_dataset import SegmentationDataset


def get_loaders(
    images: List[Path],
    masks: List[Path],
    image_arr_path: str,
    mask_arr_path: str,
    random_state: int,
    valid_size: float = 0.1,
    batch_size: int = 12,
    num_workers: int = 4,
    im_size: tuple = (480, 640),
) -> dict:

    """Creates Torch dataloaders"""

    indices = np.arange(len(images))

    # Split training and validation data
    train_indices, valid_indices = train_test_split(
        indices, test_size=valid_size, random_state=random_state, shuffle=True
    )

    np_images = np.array(images)
    np_masks = np.array(masks)

    # Establish datasets within dataset class
    train_dataset = SegmentationDataset(image_arr_path, mask_arr_path, im_size)
    train_dataset.images = np_images[train_indices]
    train_dataset.masks = np_masks[train_indices]

    valid_dataset = SegmentationDataset(image_arr_path, mask_arr_path, im_size)
    valid_dataset.images = np_images[valid_indices]
    valid_dataset.masks = np_masks[valid_indices]

    # Generate loaders for both datasets
    train_loader = DataLoader(
        train_dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=num_workers,
        drop_last=False,
    )

    valid_loader = DataLoader(
        valid_dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=num_workers,
        drop_last=False,
    )

    loaders = collections.OrderedDict()
    loaders["train"] = train_loader
    loaders["valid"] = valid_loader

    return loaders

# Import dependencies
import json
import numpy as np
import matplotlib.pyplot as plt
import argparse
import pickle
import cv2

# Import utilities for Torch
from typing import List
from pathlib import Path
import torch
from torch.utils.data import Dataset
import collections
from torch.utils.data import DataLoader

# Import model building and training utilities
import segmentation_models_pytorch as smp
from torch import nn
from catalyst.contrib.nn import DiceLoss, IoULoss
from torch import optim
from catalyst import utils
from torch.optim import AdamW

from catalyst.contrib.nn import RAdam, Lookahead
from catalyst.dl import SupervisedRunner

from catalyst.dl.callbacks import (
    DiceCallback,
    IouCallback,
    CriterionCallback,
    AccuracyCallback,
    MulticlassDiceMetricCallback,
)

# Import dataset and data loaders
from segmentation_dataset import SegmentationDataset
from data_loaders import get_loaders
import helper_operations
from helper_operations import CrossentropyND, DC_and_CE_loss

# Enable argument parsing for file paths
ap = argparse.ArgumentParser()

ap.add_argument(
    "-train_images",
    "--train_images",
    required=True,
    help="path to train images .npy file",
)
ap.add_argument(
    "-train_masks", "--train_masks", required=True, help="path to train masks .npy file"
)
ap.add_argument(
    "-test_images", "--test_images", required=True, help="path to test images .npy file"
)
ap.add_argument(
    "-test_masks", "--test_masks", required=True, help="path to test mask .npy file"
)

args = vars(ap.parse_args())

train_images_path = args["train_images"]
train_masks_path = args["train_masks"]
test_images_path = args["test_images"]
test_masks_path = args["test_masks"]

# Set a seed for reproducibility
SEED = 42
utils.set_global_seed(SEED)
utils.prepare_cudnn(deterministic=True)

# Set up U-Net with pretrained EfficientNet backbone

ENCODER = "efficientnet-b3"
ENCODER_WEIGHTS = "imagenet"
DEVICE = "cuda"

ACTIVATION = None

model = smp.Unet(
    encoder_name=ENCODER,
    encoder_weights=ENCODER_WEIGHTS,
    classes=3,
    activation=ACTIVATION,
)

# Get Torch loaders
loaders = get_loaders(
    images=np.load(train_images_path),
    masks=np.load(train_masks_path),
    image_arr_path=train_images_path,
    mask_arr_path=train_masks_path,
    random_state=420,
    valid_size=0.1,
    batch_size=3,
    num_workers=2,
)

# Optimize for cross entropy using Adam
criterion = {
    "CE": CrossentropyND(),
}

learning_rate = 0.001
encoder_learning_rate = 0.0005
encoder_weight_decay = 0.00003
optimizer_weight_decay = 0.0003
optim_factor = 0.25
optim_patience = 2

optimizer = AdamW(
    model.parameters(),
    lr=0.001,
    betas=(0.9, 0.999),
    eps=1e-08,
    weight_decay=0.01,
    amsgrad=False,
)

scheduler = optim.lr_scheduler.ReduceLROnPlateau(
    optimizer, factor=optim_factor, patience=optim_patience
)

num_epochs = 10
device = utils.get_device()

runner = SupervisedRunner(device=device, input_key="image", input_target_key="mask")

# Use Catalyst callbacks for metric calculations during training
callbacks = [
    CriterionCallback(input_key="mask", prefix="loss", criterion_key="CE"),
    MulticlassDiceMetricCallback(input_key="mask"),
]

# Train and print model training logs
runner.train(
    model=model,
    criterion=criterion,
    optimizer=optimizer,
    scheduler=scheduler,
    loaders=loaders,
    callbacks=callbacks,
    logdir="content/full_model2",
    num_epochs=num_epochs,
    main_metric="loss",
    minimize_metric=True,
    fp16=None,
    verbose=True,
)

# Test model on test dataset
test_data = SegmentationDataset(test_images_path, test_masks_path)

infer_loader = DataLoader(test_data, batch_size=12, shuffle=False, num_workers=4)
# Get model predictions on test dataset
predictions = np.vstack(
    list(
        map(
            lambda x: x["logits"].cpu().numpy(),
            runner.predict_loader(
                loader=infer_loader, resume=f"content/full_model2/checkpoints/best.pth"
            ),
        )
    )
)

# Pick sample images to analyze results
low = 1
high = len(predictions) - 1
num_results = 3
num_rand_results = num_results - 2
rand_nums = np.random.randint(low, high, num_rand_results)

"""
The results specifically include images 30 and 141 as
they are images containing multiple lines and barrels.
Hence, they would be strong indicators of performance.
"""
rand_nums = np.insert(rand_nums, 0, 30)
rand_nums = np.insert(rand_nums, 0, 141)

# Save the image, mask, and predicted mask for sample test data

for num in rand_nums:
    # Show and save image
    image = np.asarray(test_data[num]["image"])
    image = np.swapaxes(image, 2, 0)
    image = np.swapaxes(image, 1, 0)
    image = image.astype(np.uint8)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_label = "Raw_Image_{}".format(num)
    plt.imsave(image_label + ".png", image)

    # Show and save mask
    mask = np.squeeze(test_data[num]["mask"])
    mask_label = "Ground_Truth_{}".format(num)
    plt.imsave(mask_label + ".png", mask)

    # Show and save predicted mask
    pred_mask = np.asarray(predictions[num])
    pred_mask = np.swapaxes(pred_mask, 2, 0)
    pred_mask = np.swapaxes(pred_mask, 1, 0)
    pred_mask = pred_mask.astype(np.float64)
    pred_mask = np.argmax(pred_mask, axis=2)
    pred_mask_label = "Predicted_Mask_{}".format(num)
    plt.imsave(pred_mask_label + ".png", pred_mask)

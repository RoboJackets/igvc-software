# Dependencies
import json
import numpy as np
import matplotlib.pyplot as plt
import argparse

# Torch utilities
from typing import List
from pathlib import Path
import torch
from torch.utils.data import Dataset
import collections
from torch.utils.data import DataLoader

# Model building and training
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

# Dataset and data loaders
from segmentation_dataset import SegmentationDataset
from data_loaders import get_loaders
import helper_operations

# Argument parsing for file paths
ap = argparse.ArgumentParser()

ap.add_argument(
    "-a", "--train_images", required=True, help="path to train images .npy file"
)
ap.add_argument(
    "-b", "--train_masks", required=True, help="path to train masks .npy file"
)
ap.add_argument(
    "-c", "--test_images", required=True, help="path to test images .npy file"
)
ap.add_argument("-d", "--test_masks", required=True, help="path to test mask .npy file")

args = vars(ap.parse_args())

train_images_path = str((args['train_images']))
train_masks_path = str((args['train_masks']))
test_images_path = str((args['test_images']))
test_masks_path = str((args['test_masks']))

# Sets a seed for better reproducibility
SEED = 42
utils.set_global_seed(SEED)
utils.prepare_cudnn(deterministic=True)

# Sets up transfer learning system

ENCODER = "efficientnet-b3"
ENCODER_WEIGHTS = "imagenet"
DEVICE = "cuda"

# ACTIVATION = 'softmax'
ACTIVATION = None

model = smp.Unet(
    encoder_name=ENCODER,
    encoder_weights=ENCODER_WEIGHTS,
    classes=3,
    activation=ACTIVATION,
)

# Runs data loaders
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

# Optimizes for cross entropy using Adam
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

# Uses Catalyst callbacks for metric calculations
callbacks = [
    CriterionCallback(input_key="mask", prefix="loss", criterion_key="CE"),
    MulticlassDiceMetricCallback(input_key="mask"),
]

# Trains and prints training logs for model
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
    monitoring_params=None,
    verbose=True,
)

# Test model on test dataset
test_data = SegmentationDataset(test_images_path, test_masks_path)
infer_loader = DataLoader(test_data, batch_size=12, shuffle=False, num_workers=4)

# Generates predictions on test data
predictions = runner.predict_loader(
    model=model,
    loader=infer_loader,
    resume=f"content/full_model2/checkpoints/best.pth",
    verbose=False,
)

# Show an image from the test dataset
show_image = np.asarray(test_data[30]["image"])
show_image = np.swapaxes(show_image, 2, 0)
show_image = np.swapaxes(show_image, 1, 0)
show_image = show_image.astype(np.uint8)
np.shape(show_image)
plt.imshow(show_image)

# Show model prediction for image
show_image = np.asarray(predictions[30])
show_image = np.swapaxes(show_image, 2, 0)
show_image = np.swapaxes(show_image, 1, 0)
show_image = show_image.astype(np.uint8)
np.shape(show_image)
plt.imshow(show_image)

# Import dependencies
import json
import numpy as np
import matplotlib.pyplot as plt
import argparse
import pickle
import cv2
from tqdm import tqdm
import yaml
import os

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
import catalyst
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
from data_utils.segmentation_dataset import SegmentationDataset
from data_utils.data_loaders import get_loaders
import train_utils.helper_operations
from train_utils.helper_operations import CrossentropyND, DC_and_CE_loss
from train_utils.get_args import get_args
from train_utils.save import save_result


def main():
    # Enable argument parsing for file paths
    args = vars(get_args())

    train_images_path = args["train_images"]
    train_masks_path = args["train_masks"]
    test_images_path = args["test_images"]
    test_masks_path = args["test_masks"]

    # print out yaml file configuration
    dir_path = os.path.dirname(os.path.realpath(__file__))
    yaml_path = os.path.join(dir_path, "config/igvc.yaml")
    ARCH = yaml.safe_load(open(yaml_path, "r"))

    # Set a seed for reproducibility
    utils.set_global_seed(ARCH["train"]["seed"])
    utils.prepare_cudnn(deterministic=ARCH["train"]["cudnn"])

    # Set up U-Net with pretrained EfficientNet backbone
    model = smp.Unet(
        encoder_name=ARCH["encoder"]["name"],
        encoder_weights=ARCH["encoder"]["weight"],
        classes=ARCH["train"]["classes"],
        activation=ARCH["encoder"]["activation"],
    )

    # Get Torch loaders
    loaders = get_loaders(
        images=np.load(train_images_path),
        masks=np.load(train_masks_path),
        image_arr_path=train_images_path,
        mask_arr_path=train_masks_path,
        random_state=ARCH["train"]["random_state"],
        valid_size=ARCH["train"]["valid_size"],
        batch_size=ARCH["train"]["batch_size"],
        num_workers=ARCH["train"]["num_workers"],
    )

    # Optimize for cross entropy using Adam
    criterion = {
        "CE": CrossentropyND(),
    }

    optimizer = AdamW(
        model.parameters(),
        lr=ARCH["train"]["lr"],
        betas=(ARCH["train"]["betas_min"], ARCH["train"]["betas_max"]),
        eps=float(ARCH["train"]["eps"]),
        weight_decay=ARCH["train"]["w_decay"],
        amsgrad=ARCH["train"]["amsgrad"],
    )

    scheduler = optim.lr_scheduler.ReduceLROnPlateau(
        optimizer,
        factor=ARCH["train"]["optim_factor"],
        patience=ARCH["train"]["optim_patience"],
    )

    device = utils.get_device()
    print("Using device: {}".format(device))
    print(f"torch: {torch.__version__}, catalyst: {catalyst.__version__}")

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
        logdir=ARCH["train"]["logdir"],
        num_epochs=ARCH["train"]["epochs"],
        main_metric="loss",
        minimize_metric=ARCH["train"]["minimize_metric"],
        fp16=ARCH["train"]["fp16"],
        verbose=ARCH["train"]["verbose"],
    )

    # Test model on test dataset
    test_data = SegmentationDataset(test_images_path, test_masks_path)
    infer_loader = DataLoader(
        test_data,
        batch_size=ARCH["test"]["batch_size"],
        shuffle=ARCH["test"]["shuffle"],
        num_workers=ARCH["test"]["num_workers"],
    )

    # Get model predictions on test dataset
    predictions = np.vstack(
        list(
            map(
                lambda x: x["logits"].cpu().numpy(),
                runner.predict_loader(
                    loader=infer_loader,
                    resume=f"content/full_model2/checkpoints/best.pth",
                ),
            )
        )
    )

    save_result(predictions, test_data)


if __name__ == "__main__":
    main()

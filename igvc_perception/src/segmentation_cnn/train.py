# Dependencies

# Data handling
import json
import numpy as np
import matplotlib.pyplot as plt

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

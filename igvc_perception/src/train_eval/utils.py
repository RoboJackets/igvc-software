import numpy as np
from math import floor
import argparse

# Opens configuration file for locations of training related files.
# Paths are delimited by '='.
def read_data_cfg(datacfg):
    options = dict()
    with open(datacfg, "r") as fp:
        lines = fp.readlines()

    for line in lines:
        line = line.strip()
        if line == "" or line[0] == "#":
            continue
        key, value = line.split("=")
        key = key.strip()
        value = value.strip()
        options[key] = value
    return options


def get_args():
    # Training settings.
    parser = argparse.ArgumentParser(description="IGVC segmentation of lines.")

    # Hyperparameters.
    parser.add_argument(
        "--batch_size", type=int, default=1, help="input batch size for training."
    )
    parser.add_argument(
        "--val_size", type=int, default=80, help="input batch size for validation."
    )
    parser.add_argument(
        "--epochs", type=int, default=5, help="number of epochs to train"
    )
    parser.add_argument(
        "--im_size",
        type=int,
        nargs=3,
        default=[3, 400, 400],
        help="image dimensions for training.",
    )
    parser.add_argument(
        "--kernel_size",
        type=int,
        default=3,
        help="size of convolution kernels/filters.",
    )
    parser.add_argument("--lr", type=float, default=1e-3, help="learning rate.")
    parser.add_argument(
        "--lr_decay", type=float, default=1.0, help="Learning rate decay multiplier."
    )
    parser.add_argument(
        "--step_interval",
        type=int,
        default=100,
        help="Update learning rate every <step_interval> epochs.",
    )
    parser.add_argument(
        "--weight_decay", type=float, default=0.0, help="Weight decay hyperparameter."
    )

    # Other configuration.
    parser.add_argument(
        "--save_model", action="store_true", default=False, help="Save pytorch model."
    )
    parser.add_argument(
        "--save_interval",
        type=int,
        default=1,
        help="Save pytorch model after <save_interval> epochs.",
    )
    parser.add_argument(
        "--load_model",
        type=str,
        default=None,
        help="Load model from .pt file, either for initialization or evaluation.",
    )
    parser.add_argument(
        "--log_interval",
        type=int,
        default=10,
        help="number of batches between logging train status.",
    )
    parser.add_argument(
        "--vis",
        action="store_true",
        default=False,
        help="Visualize model output every log interval.",
    )
    parser.add_argument(
        "--no_cuda", action="store_true", default=False, help="disables CUDA training"
    )
    parser.add_argument(
        "--seed", type=int, default=1, metavar="S", help="random seed (default: 1)"
    )
    parser.add_argument(
        "--cfgfile",
        type=str,
        default="cfg/igvc.cfg",
        help="Directory containing cfg for train and evaluation.",
    )
    parser.add_argument(
        "--test",
        action="store_true",
        default=False,
        help="Skip training, and evaluate a loaded model on test data.",
    )
    parser.add_argument(
        "--val_samples",
        type=int,
        default=10,
        help="Number of validation samples to use from train data.",
    )
    return parser.parse_args()

import argparse


def get_args():
    # Training settings.
    ap = argparse.ArgumentParser(description="IGVC multi-class segmentation.")
    ap.add_argument(
        "-train_images",
        "--train_images",
        required=True,
        help="path to train images .npy file",
    )
    ap.add_argument(
        "-train_masks",
        "--train_masks",
        required=True,
        help="path to train masks .npy file",
    )
    ap.add_argument(
        "-test_images",
        "--test_images",
        required=True,
        help="path to test images .npy file",
    )
    ap.add_argument(
        "-test_masks",
        "--test_masks",
        required=True,
        help="path to test mask .npy file",
    )
    return ap.parse_args()

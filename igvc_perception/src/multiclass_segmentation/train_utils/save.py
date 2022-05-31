import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np
from tqdm import tqdm
import cv2
import os


def save_result(predictions, test_data, save_path):
    # Pick sample images to analyze results
    low = 1
    high = len(predictions) - 1
    num_results = 30
    num_rand_results = num_results - 2
    rand_nums = np.random.randint(low, high, num_rand_results)

    """
    The results specifically include images 30 and 141 as
    they are images containing multiple lines and barrels.
    Hence, they would be strong indicators of performance.
    Can add certain samples specific to datasets
    """
    # rand_nums = np.insert(rand_nums, 0, 30)
    # rand_nums = np.insert(rand_nums, 0, 141)

    # Save the image, mask, and predicted mask for sample test data
    predictions_path = os.path.join(save_path, "predictions")
    Path(predictions_path).mkdir(parents=True, exist_ok=True)
    for num in tqdm(rand_nums):
        fig = plt.figure(figsize=(10, 4))
        # Show and save image
        plt.subplot(1, 3, 1)
        image = np.asarray(test_data[num]["image"])
        image = np.swapaxes(image, 2, 0)
        image = np.swapaxes(image, 1, 0)
        image = image.astype(np.uint8)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        plt.imshow(image)
        plt.gca().set_title("Raw Image")

        # Show and save mask
        plt.subplot(1, 3, 2)
        mask = np.squeeze(test_data[num]["mask"])
        plt.imshow(mask)
        plt.gca().set_title("Ground Truth")

        # Show and save predicted mask
        plt.subplot(1, 3, 3)
        pred_mask = np.asarray(predictions[num])
        pred_mask = np.swapaxes(pred_mask, 2, 0)
        pred_mask = np.swapaxes(pred_mask, 1, 0)
        pred_mask = np.argmax(pred_mask, axis=2)
        plt.imshow(pred_mask)
        plt.gca().set_title("Predicted")

        plt.tight_layout()
        save_path = os.path.join(predictions_path, f"{num}.png")
        plt.savefig(save_path)
        plt.close()

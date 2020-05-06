# Import dependencies

from sklearn.model_selection import train_test_split
import numpy as np
import argparse

# Set size and seed

test_size = 0.2
random_state = 42

# Establish arguments as file paths
ap = argparse.ArgumentParser()

ap.add_argument("-images", "--images", required=True, help="images npy location")
ap.add_argument("-masks", "--masks", required=True, help="masks npy location")
args = vars(ap.parse_args())

images_path = args["images"]
masks_path = args["masks"]

# Load images and masks as numpy arrays
X = np.load(images_path)
Y = np.load(masks_path)

# Split the numpy arrays and save as .npy files
X_train, X_test, Y_train, Y_test = train_test_split(
    X, Y, test_size=test_size, random_state=random_state
)

np.save("train_images.npy", X_train)
np.save("test_images.npy", X_test)

np.save("train_masks.npy", Y_train)
np.save("test_masks.npy", Y_test)

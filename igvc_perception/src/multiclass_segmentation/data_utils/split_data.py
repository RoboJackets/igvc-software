# Import dependencies
from sklearn.model_selection import train_test_split
import numpy as np
import argparse
import os

if __name__ == "__main__":
    # Set size and seed
    test_size = 0.2
    random_state = 42
    X = np.array([])
    Y = np.array([])
    # Establish arguments as file paths
    ap = argparse.ArgumentParser()

    ap.add_argument("-dir", "--dir", required=True,
        help="igvc_dataset location")
    args = vars(ap.parse_args())

    # Load images and masks as numpy arrays
    for dir_name in os.listdir(args["dir"]):
        data_path = os.path.join(args["dir"], dir_name)

        for data_name in os.listdir(data_path):
            if data_name == "masks.npy":
                masks_path = os.path.join(data_path, data_name)
                data_npy = np.load(masks_path)
                if X.shape[0] == 0:
                    X = data_npy
                else:
                    X = np.vstack((X, data_npy))
                    # X = np.concatenate((X,data_npy),axis=0)

            elif data_name == "images.npy":
                images_path = os.path.join(data_path, data_name)
                data_npy = np.load(images_path)
                if Y.shape[0] == 0:
                    Y = data_npy
                else:
                    Y = np.vstack((Y, data_npy))
                    # Y = np.concatenate((Y,data_npy),axis=0)

    # Split the numpy arrays and save as .npy files
    labels_train, labels_test, images_train, images_test = train_test_split(
        X, Y, test_size=test_size, random_state=random_state
    )

    np.save("data/train_images.npy", images_train)
    np.save("data/test_images.npy", images_test)

    np.save("data/train_masks.npy", labels_train)
    np.save("data/test_masks.npy", labels_test)

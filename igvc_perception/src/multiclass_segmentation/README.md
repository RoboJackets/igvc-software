# Multiclass Semantic Segmentation System

Using transfer learning, we can simultaneously classify lines and barrels in images. A U-Net with a pretrained EfficientNet encoder performs multiclass semantic segmentation, classifying the pixels in each image as a line, barrel, or neither. This system uses the [Catalyst](https://github.com/catalyst-team/catalyst) and [Segmentation_Models](https://github.com/qubvel/segmentation_models.pytorch) libraries. 

<p align="center">
  <img src="https://github.com/suhacker1/igvc-software/blob/multiclass_segmentation/igvc_perception/src/multiclass_segmentation/combine_images.png">
</p>

## Folder Structure 
+ **IGVC_Full_Dataset.ipynb**: implements the training and testing of the neural network in a notebook
+ **IGVC_Full_Dataset.pdf**: provides a PDF version of the aforementioned notebook for readability
+ **data_loaders.py**: defines the Torch data loaders
+ **helper_operations.py**: defines multiple operations used for loss and metric calculations
+ **make_dataset.py**: converts folders of images and masks into .npy files
+ **requirements.txt**: lists dependencies
+ **segmentation_dataset.py**: defines the utilized dataset
+ **train.py**: implements, trains, and tests the model

## Build Instructions 

1. Download the dependencies in requirements.txt manually or using `pip install -r requirements.txt`. 
2. Obtain a dataset containing the images (PNGs) and masks (JSONs). Skip steps #2 and #3 if using .npy files. 
3. Run `python make_dataset.py -a /path_to_images(png) -b /path_to_masks(json)`to generate .npy files. 
4. Establish train_images.npy, train_masks.npy, test_images.npy, and test_masks.npy files. 
5. Run `python train.py -a /path_to_train_images -b /path_to_train_masks -c /path_to_test_images -d /path_to_test_masks` to train and test the neural network.

Alternatively, download **IGVC_Full_Dataset.ipynb**, and run it in [Google Colaboratory](https://colab.research.google.com/notebooks/intro.ipynb#recent=true) or a local Jupyter environment. 

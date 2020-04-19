# Multiclass Semantic Segmentation System

Using transfer learning, we can simultaneously classify lines and barrels in images. A U-Net with a pretrained EfficientNet encoder performs multi class semantic segmentation, classifying the pixels in each image as a line, barrel, or neither. This implementation is based upon the Catalyst and Segmentation-Models-Pytorch libraries. 

## Folder Structure 

## Build Instructions 

1. Download the dependencies in requirements.txt manually or using `pip install -r requirements.txt`. 
2. Obtain a dataset containing the images (PNGs) and masks (JSONs). Skip steps #2 and #3 if using .npy files. 
3. Run `python make_dataset.py -a /path_to_images(png) -b /path_to_masks(json)`to generate .npy files. 
4. Establish train_images.npy, train_masks.npy, test_images.npy, and test_masks.npy files. 
5. Run `python train.py -a /path_to_train_images -b /path_to_train_masks -c /path_to_test_images -d /path_to_test_masks` to train and test the neural network.

Alternatively, download `IGVC_Full_Dataset.ipynb` and run it in Google Colaboratory. 

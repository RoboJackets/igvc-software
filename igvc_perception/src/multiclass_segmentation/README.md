# Multiclass Semantic Segmentation System

Using transfer learning, we can simultaneously classify lines and barrels in images. A U-Net with a pretrained EfficientNet encoder performs multi class semantic segmentation, classifying the pixels in each image as a line, barrel, or neither. This implementation is based upon the Catalyst and Segmentation-Models-Pytorch libraries. 

### Installation 
1. Download the dependencies in requirements.txt manually or using `pip install -r requirements.txt`. 

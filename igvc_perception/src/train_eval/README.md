# Binary Segmentation Model Setup

For the 2019 IGVC competition, the detection  of  lines  and  potholes  was performed using a state-of-the-art  U-Net  CNN  for  Image Segmentation instead of a pretrained FCN8 to reduce model size and increase efficiency to cope with the increased loads of three total cameras.  The model is general enough to detect both potholes and lines, as the two features are similar.  Similar to the FCN8, the U-Net outputs a pixel-wise classification that can be visualized as a greyscale image.

## Setup
First generate the train.txt and test.txt files used for training the U-Net. Use ```igvc_perception/src/train_eval/train_test_generation.py``` to generate these files.
- In ```train_test_generation.py``` make sure to set the location of the dataset path on line 4.

  - Ex: ```dataset_path = '/home/vivek/Desktop/binaryTraining'```


In ```igvc_perception/src/train_eval/cfg/igvc.cfg``` input the locations of the generated train.txt and test.txt files as well as the desired location of model backups. Example:
```py
train = /home/vivek/catkin_ws/src/igvc-software/igvc_perception/src/train_eval/train.txt
test  = /home/vivek/catkin_ws/src/igvc-software/igvc_perception/src/train_eval/test.txt
backup = /home/vivek/Desktop/trainingData
```

In ```/igvc_perception/src/train_eval/train_test_generation.py``` set the path of the dataset on line 10. Example:
```py
dataset_path = '/home/vivek/Desktop/binaryTraining'
```

If using the [labelme tool](https://github.com/wkentaro/labelme) to generate json annotations use the the following [script](https://github.com/wkentaro/labelme/blob/master/examples/semantic_segmentation/labelme2voc.py) to convert to a VOC-format dataset. Download the [script](https://github.com/wkentaro/labelme/blob/master/examples/semantic_segmentation/labelme2voc.py) and place it in the directory containing your labeled dataset. Additionally make sure to add a labels.txt file which specifies the different classes that the model predicts. In this case, the model is only predicting background and line. The contents of labels.txt should look like this:
```bash
__ignore__
_background_
line
```

Example File Structure:
```bash
training-data
├── labelme2voc.py
├── labels.txt
├── dataset1
│   ├── 00000000.jpg
│   ├── 00000000.json
│   ├── 00000001.jpg
│   ├── 00000001.json
```

Now run the script which produces .png masks, .npy masks, and a visualization of the labels. Command following the example file structure mentioned above:
```bash
python3 labelme2voc.py dataset1 data_dataset_voc_dataset1 --labels labels.txt 
```
The file structure should now look like this:
```bash
training-data
├── labelme2voc.py
├── labels.txt
├── dataset1
│   ├── 00000000.jpg
│   ├── 00000000.json
│   ├── 00000001.jpg
│   ├── 00000001.json
├── data_dataset_voc_dataset1
│   ├── JPEGImages
│   │   ├── 00000000.jpg
│   │   ├── 00000001.jpg
│   ├── SegmentationClass
│   │   ├── 00000000.npy
│   │   ├── 00000001.npy
│   ├── SegmentationClassPNG
│   │   ├── 00000000.png
│   │   ├── 00000001.png
│   ├── SegmentationClassVisualization
│   │   ├── 00000000.jpg
│   │   ├── 00000001.jpg
│   ├── class_names.txt
```
Once this is done make sure to copy the files in SegmentationClassPNG to the original dataset specified in ```train_test_generation.py``` so the training detects the labels. The resultant file structure should something like this:
```bash
training-data
├── labelme2voc.py
├── labels.txt
├── dataset1
│   ├── 00000000.jpg
│   ├── 00000000.png
│   ├── 00000000.json
│   ├── 00000001.jpg
│   ├── 00000001.png
│   ├── 00000001.json
```

## Training the Model
Run ```train.py``` to begin the training. Recommended parameters:

```python3 train.py --epoch 100 --save_model --save_interval 10```

## Viewing Training Progress:
Make sure tensorboard is installed and you are in the ```/igvc_perception/src/train_eval``` directory before running:

```tensorboard --logdir=runs```

To view the training progress of the U-Net. You can example the accuracy and loss of the model to choose the best model.


## Evaluating Model
To view the performance of the model on a training set make sure you are in the ```/igvc_perception/src/train_eval``` directory and run ```evaluate.py```. Make sure the location of the model, the image directory, the annotation directory, and the visualization settings are all specified:

```
python3 evaluate.py --load_model ~/Desktop/trainingData/IGVCModel_10.pt --img_dir ~/Desktop/trainingData/data_dataset_voc_parkinglot4/JPEGImages --anno_dir ~/Desktop/trainingData/data_dataset_voc_parkinglot4/SegmentationClassPNG/ --vis
```


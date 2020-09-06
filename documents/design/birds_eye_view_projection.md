# Project Title

*Issue #665*

**Author:**
- Vivek Mhatre

## The Problem

In line_layer, we map each pixel to a grip map "cell". Since distances are not constant in the image space due to perspective warping, there are lots of "holes" after the projection from pixel to grid map "cell". Currently, opening and closing morphological operations are used to fix these "holes". It is important to create a better solution to fix these holes so we can have a better understanding of the depth in a given image and improve overall perception.

## Proposed Solution

Instead of using opening and closing morphological operations, a better way of fixing these "holes" is to use inverse projection mapping. By shrinking the bottom of the raw birds-eye view image while keeping the top of the image unchanged, inverse projection mapping preserves all available pixels from the top of the image where there is lower relative resolution.

Steps to solve issue:
1. Implement inverse projection mapping using OpenCV in Python.
2. Use the training images on the robojackets cloud to tune parameters.
3. Then I will implement my algorithm using C++ and implement IPM with a test node.
4. Implement IPM in line_layer.
5. Test the performance using ros bag files.

## Questions & Research

Questions:
- What are the parameters of the camera? Imaging sensor height and width? Effective field of view in the horizontal and vertical direction?
- IPM assumes the world is flat but how will it handle large elevation changes?

Useful Reading:
- [Paper describing use of IPM to determine position of another moving robot](https://www.scitepress.org/papers/2018/69300/69300.pdf)




Are there things you are unsure about or don't know? What do you need to research to be able to
complete this project? If you need information from the mechanical or electrical subteams,
be sure to describe that here. If your solution requires more research to implement, descibe
what kind of topics you need look at. Link any research papers/articles that you find here.

**Of course, if you have _any_ questions or concerns, feel free to bring them up at any time.
If you ever feel lost or don't know what to do or work on, Oswin or any old software member
can help you out. You are not expected to know every technical detail about the project. Expressing
what you don't know will make it easier to do research and ask for help.** 

## Overall Scope

### Affected Packages

- I will need the OpenCV package and line\_layer.cpp. I will be making changes to the igvc\_navigation package.


### Schedule

Subtask 1 (September 9th): Implement IPM in Python.

Subtask 2 (September 16th): Implement IPM in C++ on test node

Subtask 3 (September 20th): Implement IPM in line\_layer.cpp and remove morphological opening and closing.

Code Review (September 23rd): Hopefully I'll be done now.

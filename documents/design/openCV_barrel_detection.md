# Barrel Detection OpenCV

*Issue #Number*
#536

**Author:**
- Paul Case

## The Problem

Want to create a backup and/or baseline for barrel detection using a non machine learning
technique. this should be more of a reference to the neural net. I would mostly just need the
three cameras, as well as the OpeCV Library. I think that a HSV threshold would probably be sufficient.

## Proposed Solution

- How are you going to solve this problem?
- What are the steps you need to take to complete the project?
- Break down the problem above into smaller, individual components with specific metrics of success
- You can think of this section as a quantitative description of the project
- Each bullet should represent a single action
    - Use sub-bullets to provide more details and justifications for each step

I figure that using a HSV filter and along with a gaussian blur would work to make a sufficient mask for image. The algorithm would probably go like this:
1. turn image to a Mat
2. gaussian blur by 6 radius
- will change radius
3. convert to HSV
4. threshold to include orange(5-30 hue, high saturation, all value)
- obviously changable, just something I guessed
5. erode anything extra
6. close any gaps
7. lay over the mask onto the mat and send along
this should block out the orange from the image, and the blur and closing should get the white tape.


## Questions & Research

Are there things you are unsure about or don't know? What do you need to research to be able to
complete this project? If you need information from the mechanical or electrical subteams,
be sure to describe that here. If your solution requires more research to implement, describe
what kind of topics you need look at. Link any research papers/articles that you find here.

**Of course, if you have _any_ questions or concerns, feel free to bring them up at any time.
If you ever feel lost or don't know what to do or work on, Oswin or any old software member
can help you out. You are not expected to know every technical detail about the project. Expressing
what you don't know will make it easier to do research and ask for help.**

not too many questions, should be simple. One question I do have is if these barrels are guaranteed to be orange. If not, I might need to add more masks.

## Overall Scope

### Affected Packages

I will need the openCV package, as well as line_layer.cpp to put my code in



### Schedule

Subtask 1 (November 20th): I should put up a design Document

Subtask 2 (November 24th): I want to have a "rough draft" of an algorithm for the mask finished by then,

Subtask 3 (December 1st): I want to have bug tested on the rosbag I have on my ssd.

Code Review (January 5th): Putting up a PR by this date
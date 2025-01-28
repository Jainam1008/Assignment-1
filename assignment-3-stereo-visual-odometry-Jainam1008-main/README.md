[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/0UsYnfk3)
# Robot Perception - Stereo Visual Odometry

### Deadline : October 11th, 2024 11:59pm

***This class activity is to be done as individuals, not with partners nor with teams.***

### How to get started →

Begin by reading this entire writeup and making sure you have a good understanding of it. Next, spend some time planning how you’re going to approach the problem. 

# Introduction

---

In this class activity, you will implement the stereo visual odometry algorithm to estimate the motion of a robot using stereo images. You will start by implementing the stereo visual odometry algorithm using the OpenCV library. You will then test your implementation on a dataset of stereo images and evaluate the performance of your algorithm.

Stereo visual odometry is the process of estimating the motion of a robot using stereo images. The algorithm estimates the motion of a robot by tracking the features in the stereo images and computing the relative pose between the camera frames. This is then used to estimate the motion of the robot.


Algorithm Steps:
![SVO Algo](image-1.png)

# Activity Information

---

### Objectives

- Learn how visual odometry can be used to estimate the motion of a robot
- Implement the stereo visual odometry algorithm using the OpenCV library
- Test the implementation on a dataset of stereo images

### Resources

- [Visual Odometry Tutorial](https://rpg.ifi.uzh.ch/docs/VO_Part_I_Scaramuzza.pdf)
- [Epipolar Geometry](https://learnopencv.com/introduction-to-epipolar-geometry-and-stereo-vision/
)

### Requirements

- Your python script should run without errors
- Your code should be well documented
- Your code should NOT download the dataset from the internet (we will have a copy of the dataset)
- Your code should NOT use any external libraries other than OpenCV, Numpy, and Matplotlib
  
### What we provide

- Detailed instructions to download the dataset
- A template python script with the necessary imports and function definitions
- A test script to check if your implementation is correct


### The KITTI Dataset

The KITTI dataset is a popular dataset for visual odometry and SLAM research. The dataset contains stereo images, camera calibration parameters, and ground truth poses for the stereo images. You will use the KITTI dataset to test your implementation of the stereo visual odometry algorithm.

You can download the KITTI dataset from the following link:

https://www.cvlibs.net/datasets/kitti/eval_odometry.php

Make sure you download only the gray images and the calibration files for the stereo camera. You will use the gray images to test your implementation of the stereo visual odometry algorithm.

![KITTI-Screenshot](image.png)

You must also download the ground truth poses for the stereo images. The ground truth poses are provided in a text file with the following format:

Each row of the file contains the first 3 rows of a 4x4 homogeneous pose matrix flattened into one line.
It means that this matrix:

```
r11 r12 r13 tx
r21 r22 r23 ty
r31 r32 r33 tz
0   0   0   1
```

is represented in the file as a single row: `r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz`


This means you will only need the `tx` and `tz` values from the ground truth poses to evaluate the performance of your algorithm.

**Note: You will need to parse the ground truth poses file to extract the `tx` and `tz` values. Also, your visual odometry estimation is only limited to 2D transformations**

### What to submit

You must submit a package with the follow file structure


```bash
visual_odometry
├── scripts
│   └── test_ATE.py # we will provide this script soon
│   └── visual_odometry.py # Your implementation
└── config.yaml
```

Your code should accept the dataset path and output csv path as command-line arguments. The output directory should contain the aligned images. Use `OS` to handle file paths.

Example command to run the script: `python visual_odometry.py --config config.yaml`

The `config.yaml` file should contain the following fields:

```yaml
dataset: data_odometry_gray
sequence: 00
output: output.csv
```

For example, if you are running the script on the KITTI dataset, the dataset argument should be the path to base of the dataset, which means, running `ls -R` here should give you -
```
ls -R data_odometry_gray
dataset

data_odometry_gray/dataset:
sequences

data_odometry_gray/dataset/sequences:
00	02	04	06	08	10	12	14	16	18	20
01	03	05	07	09	11	13	15	17	19	21

data_odometry_gray/dataset/sequences/00:
calib.txt	image_0		image_1		times.txt
```

sequence should let the user choose the sequence number. The output argument should be the path to the output file where you will save the estimated poses as a csv.


*Please make sure you adhere to the structure above, if your package doesn’t match it the grader will give you a **zero***

### Grading considerations

- **Late submissions:** Carefully review the course policies on submission and late assignments. Verify before the deadline that you have submitted the correct version.
- **Environment, names, and types:** You are required to adhere to the names and types of the functions and modules specified in the release code. Otherwise, your solution will receive minimal credit.

# Part 1 : Motion Estimation (between two frames)

---

In this part, you will only estimate the motion between two stereo frames. You will use the OpenCV library to extract features from the stereo images and compute the relative pose between the camera frames. This part is crucial as it will help you understand the stereo visual odometry algorithm and how to estimate the motion of a robot using stereo images.

The template code provided to you contains various functions which you need to implement. Go over the template code and implement the functions.

To enable you to write only 1 python file, You can test the implementation of Part 1 by providing only 2 set of stereo images in the following way

```python
def main(config_path):
    """Main function for stereo visual odometry."""
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)

    path = config['dataset_path']
    sequence = config['sequence']
    max_frames = config['max_frames']
    match_threshold = config['match_threshold']
    save_csv = config.get('save_csv', False)
    csv_path = config.get('csv_path', './')

    # Load the camera intrinsic matrix

    # Load the ground truth poses

    # Load the first left and right image frames

    # Load the second left and right image frames

    # Compute translation and rotation of the second set of frames wrt the first set of frame
```


# Part 2 : Visual Odometry (between multiple frames)

---

Now that you have your basic motion estimation working, you will extend it to estimate the motion between multiple frames. Add your main loop to estimate the motion between multiple frames. 

# Submission and Assessment

---

Submit using the Github upload feature on [autolab](https://autolab.cse.buffalo.edu)

**Note: Make sure your code complies to all instructions, especially the naming conventions. Failure to comply will result in zero credit**

**Grading Scheme and penalties will be defined later**

**Note: You will not receive partial credit for files working /not working. Please make sure your scripts execute**

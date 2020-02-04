# SIFT Flow CVO

## Summary
This repository contains source codes for Contineous Visual Odometry (CVO) ruuning with SIFT-flow as adiitional labels.

## Dependency
* ubuntu 16.04
* PointCloudLibrary 1.4
* Eigen3

## Data
* Download TUM RGBD dataset from [here](https://vision.in.tum.de/data/datasets/rgbd-dataset/download).

## Generate sift flow
* Reference: [SIFT Flow: Dense Correspondence across Scenes and its Applications. Liu et. al. IEEE transactions on pattern analysis and machine intelligence 33, no. 5 (2010): 978-994.](https://people.csail.mit.edu/celiu/SIFTflow/)
* Revised code is my repository [chienerh/SIFT-Flow](https://github.com/chienerh/SIFT-Flow) which fixed minor error and generate sift flow bin file for CVO to read in.

## cpp
To compile the cpp code, type the command below:
``` 
cd dev/cpp/rkhs_se3_registration
mkdir build
cd build
```
If this is your first time compiling using intel compiler, set your cmake varaibles by the following command: ([learn more here](https://gitlab.kitware.com/cmake/community/wikis/FAQ#how-do-i-use-a-different-compiler))
```
cmake .. -DCMAKE_C_COMPILER=icc -DCMAKE_CXX_COMPILER=icpc
make
```
After that, you only need to do the following:
```
cmake ..
make
```
Then executable files ```cvo``` and ```adaptive_cvo``` will be generated in build.

To run cvo code: 
```
./cvo path_to_data tum_sequence_number(1 for fr1, 2 for fr2, 3 for fr3)
```
A txt file of the trajectory ```cvo_poses_qt.txt``` will be generated in your data folder.

To run adaptive cvo code:
```
./adaptive_cvo path_to_data tum_sequence_number(1 for fr1, 2 for fr2, 3 for fr3)
```
A txt file of the trajectory ```acvo_poses_qt.txt``` will be generated in your data folder.

## TUM Evaluation Tools
* [TUM evaluation tools](https://vision.in.tum.de/data/datasets/rgbd-dataset/tools) were modifed into python3 in our repository.
* Evaluation tools are in ```data/rgbd_dataset/rgbd_benchmark_tools/```. 

* Then  navigate to the tools folder: 
    ```cd data/rgbd_dataset/rgbd_benchmark_toos/```
    
* You can run the evaluation codes by parsing arguments into it, for more informations:
    ```python3 evaluate_ate.py -h```
    ```python3 evaluate_rpe.py -h```

* Examples:
    * ATE: 
    ```python3 evaluate_ate.py ../freiburg1_desk/groundtruth.txt ../freiburg1_desk/cvo_poses_qt.txt --verbose --plot ../freiburg1_desk/figure/ate.png --title 'fr1_desk'```
    * RPE: 
    ```python3 evaluate_rpe.py ../freiburg1_desk/groundtruth.txt ../freiburg1_desk/cvo_poses_qt.txt --fixed_delta --verbose --plot ../freiburg1_desk/figure/rpe.png --title 'fr1_desk'```

  
## Results







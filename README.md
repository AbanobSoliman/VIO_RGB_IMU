# RGB-IMU FUSION FRAMEWORK IN APPLICATION OF AGENTS LOCALIZATION IN CHALLENGING ENVIRONMENTS
![Alt text](./src/cover.png?raw=true "Body & Inertial Frames") \
In order to apply the Error-states Extended Kalman Filter Fusion framework: 
- Add the dataset from [TUM VIO datasets website](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) to the folder of path: "tum_dataset\dataset_folder_name" .
- Add the path of the "src" and "tum_dataset\dataset_folder_name" folders to your Matlab directory.
- Test the algorithm with the 3 (MAIN) Matlab scripts in the "apps" folder.
## MAIN_CAM_RealData_IMU_EKF.m
Here the RGB camera pose readings are obtained using the stochastic theory discussed in: 
> R. Jiang, R., R. Klette, and S. Wang. "Modeling of Unbounded Long-Range Drift in Visual Odometry." 2010 Fourth Pacific-Rim Symposium on Image and Video Technology. Nov. 2010, pp. 121-126.
## MAIN_CAM_SfM_IMU_EKF.m
Here the RGB camera pose readings are obtained using the increamental structure from motion algorithm.
> PS: This algorithm takes long processing time in the Sfm step.
## MAIN_CAM_synthetic_IMU_EKF.m
Here we apply our algorithm on [MATLAB synthetic dataset](https://fr.mathworks.com/help/driving/ug/visual-inertial-odometry-using-synthetic-data.html), to test the performance of our framework compared to that of MATLAB.

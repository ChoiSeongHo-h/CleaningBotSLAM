# CleaningBotSLAM
[![Video Label](http://img.youtube.com/vi/Qqig8yXCvjw/0.jpg)](https://youtu.be/Qqig8yXCvjw)

- 2022 Top Prize, Creative Comprehensive Design Competition, Hanyang Univ. ERICA
- 2022 Gold Prize, International S.M.A.R.T Sustainable Technology Competition, Korea Sanhak Foundation
- 2022 Bronze Prize, Intelligent Robotics Industry Consortium Creative Comprehensive Design Competition, Hanyang Univ. ERICA

## Overview
**Positioning module** for autonomous cleaning robots over the ocean

- **6DoF drift free pose estimation** without loop closure using GPS
- Maintaining high stability using **IMU bias divergence detection** and **divergence suppression loss** function
- **Acceleration** of front and back end of SLAM using **GPU**
- Integration with **navigation** module using **RGBD camera** and **occupancy grid OctoMap**




## 1 6DoF drift free pose estimation 
**6DoF drift free pose estimation** using robot kinematics and GPS



### 1.1 Eliminating yaw drift
[![Video Label](http://img.youtube.com/vi/CH1DeH8SLzg/0.jpg)](https://youtu.be/CH1DeH8SLzg)

The direction of robot displacement is usually the direction of the robot's heading, although there are effects from wind and water. When a series of GPS measurements are on a line, I want to force the direction of the line to be the estimated heading direction.

#### 1.1.1 Calculating the covariance of a motion sliding window for robot motion capture
I first compute the covariance of the GPS sliding window elements. If the robot is stationary and the magnitude of the covariance, or the determinant, is below a certain value, I do not use this heading alignment process. This means that the heading correction must be linear in motion and at least a certain speed.

#### 1.1.2 PCA to determine robot heading axis
I perform a PCA based on the calculated covariance above a certain size. If the component in the major axis direction is significantly larger than the component in the minor axis direction, use the major axis direction as the heading direction.

#### 1.1.3 Robust algorithm to determine robot heading direction
Once the major axis direction is determined, I determine whether the collection of GPS measurements is negatively or positively oriented. I normalize the motion vectors between each element of the GPS window and add them together. I then compare the combined vector to the direction of the major axis (dot product). This allows for very robust direction estimation.

#### 1.1.4 Eigenratio for GPS heading covariance
On the other hand, I define eigenratio as (major eigenvalue)/(minor eigenvalue). This is the relative stretch in the direction of the major axis. Find the variance of GPS heading based on the eigenratio. The assumed formula is

![화면 캡처 2022-06-14 020345](https://user-images.githubusercontent.com/72921481/173406892-0c0c9bba-4565-41b4-ac30-d41821660bdd.jpg)

If the major and minor axes are the same size and eigenration is 1, the covariance is infinitely large. Conversely, eigenration greater than 1 has a smaller covariance, i.e., the covariance is smaller when the motion is large and linear. The above process has determined the covariance and direction, which are used in the heading alignment loss function.

Additionally, the robot only trusts these estimates when roll and pitch are small.

### 1.2 Eliminating altitude drift
The altitude can be obtained from the GPS, but it is very inaccurate, so I ignore this measurement. Given the nature of our robot's motion in the ocean, I added a loss function that forces the altitude to be zero. This loss function has a large covariance to better reflect the short-term altitude changes in the visual inertial system while eliminating drift.

### 1.3 Eliminating X, Y drift
X, Y drift is eliminated by GPS.

### 1.4 Eliminating roll, pitch drift
Roll and pitch drifts are eliminated by the IMU.

## 2 IMU bias suppression  
Maintaining high stability using **IMU bias divergence detection** and **divergence suppression loss** function

### 2.1 Problems with IMU 
The IMU measurement noise and bias variation, camera parameters, and IMU-camera synchronization were all carefully calibrated, but the IMU introduced a very large drift. This was the case even when using VI-Stereo-DSO, which led me to believe that the problem was largely with the IMU itself. The bias problem was especially noticeable with the accelerometer, which was bad for both velocity and position.

### 2.2 Lack of constraints on IMU bias  
Since I couldn't change the IMU, I had to improve the optimization system to solve this problem. I believe that VINS fusion has insufficient bias constraints for poor IMUs. This is because only the covariance of the bias grows over time, and there is no constraint on the absolute size of the bias. ( Strictly speaking, the bias is constrained to be the same size as before.) This makes the bias sensitive to other states and measurements, and causes states to diverge out of the manifold.

### 2.3 IMU bias anti-divergence loss function
To address this, I added a constraint on the IMU bias. To determine the mean of the bias magnitude, I clustered the IMU measurements into 20 units and subtracted the mean from the overall mean of the measurements to calculate the bias magnitude and its variance. (The reason for clustering into 20 units was to reduce the effect of measurement noise.) Based on the mean and variance of the bias, a loss function was constructed to ensure that the three-axis bias magnitude in the SLAM system was close to the calculated mean.

The initial Jacobian was constructed as follows:
![화면 캡처 2022-06-14 014404](https://user-images.githubusercontent.com/72921481/173403588-18c2b65f-04c0-45d7-b211-df186b98af93.jpg)

In my test case, this was numerically very unstable. Since the loss function can be satisfied by adjusting the bias of only one axis, it is not possible to determine which axis should be adjusted. This makes the inverse matrix very unstable.

### 2.4 Improved loss function 
Instead of imposing constraints on all axes at once, I thought that I could improve the stability of the system by imposing constraints on each axis. Therefore, I decided to constrain the size of the bias for each axis instead of constraining the size of the overall bias. Thus, the system minimizes the Mahalanobis distance between the precomputed bias mean and the state bias magnitude for each axis.

### 2.5 With the divergence detection module
This process, together with the IMU divergence detection module, significantly increased the stability of the system. The IMU divergence detection module determines that a large difference between the GPS position and the estimated position is a divergence and resets the system. 

## 3 SLAM Acceleration 
**GPU acceleration** on the SLAM front end and back end, **modifying** the image **buffer**, and **removing loop closures** to ensure real-time. 

### 3.1 System overview
I tuned the VINS Fusion GPU lite version (https://github.com/KopiSoftware/VINS_Lite_GPU) and used it as the localization module. The basic VINS Fusion GPU lite uses GPU acceleration only for the front end, such as corner detection and optical flow. On the other hand, our system used a low-performance Jetson Nano, and there were other tasks that consumed CPU resources, so running the back end of SLAM on the CPU was not guaranteed to be real-time.

### 3.2 SLAM acceleration methods 
To ensure real-time, I used Cuda acceleration, which is supported in Ceres version 2. I also modified the image buffer to store only the most recent image. Instead of using a loop closure to reduce the drift of the heading, the improved system uses GPS. Therefore, I was able to improve the processing speed by removing unnecessary loop closures.

These system improvements resulted in faster computation. The system performed around 20 fps, which is the result of properly distributing tasks between the GPU and CPU.

## 4 Integration with navigation module
Integration with **navigation** module using **RGBD camera** and **occupancy grid OctoMap**

The navigation module compresses the data from the RGBD camera into an occupancy grid OctoMap. The compressed map constitutes a local map. The local map is projected back to 2D, and navigation is performed on this 2D map.

It is noise robust thanks to its occupancy grid map property and memory efficient thanks to its OctoMap property.

## 5 Presentation board

![2022_capstone_BWW_2_0_복사본-001](https://user-images.githubusercontent.com/72921481/173407500-e30a7176-7d6f-45de-893b-14dd2e777d9e.png)

## 6 Presentation slides

![캡스톤-최종-ppt-_page-0001](https://user-images.githubusercontent.com/72921481/173407728-d1754e88-034b-4e4e-bf9a-007004806603.jpg)
![캡스톤-최종-ppt-_page-0002](https://user-images.githubusercontent.com/72921481/173407738-4ff92026-cf65-4440-a846-8e588ee1de9f.jpg)
![캡스톤-최종-ppt-_page-0003](https://user-images.githubusercontent.com/72921481/173407739-12d6db74-ee7f-4366-9bff-51f14a0bca73.jpg)
![캡스톤-최종-ppt-_page-0004](https://user-images.githubusercontent.com/72921481/173407742-3ace72c9-1d06-43af-8ef6-bd5b5e0aeee9.jpg)
![캡스톤-최종-ppt-_page-0005](https://user-images.githubusercontent.com/72921481/173407744-85108984-7d93-4752-9ca4-8d04842628a1.jpg)
![캡스톤-최종-ppt-_page-0006](https://user-images.githubusercontent.com/72921481/173407748-52ad4a46-3ffb-430d-93a4-9a133ec0e867.jpg)
![캡스톤-최종-ppt-_page-0007](https://user-images.githubusercontent.com/72921481/173407750-05c13727-a0c0-432e-8e68-193b9ffac506.jpg)
![캡스톤-최종-ppt-_page-0008](https://user-images.githubusercontent.com/72921481/173407756-85d2075c-06e0-4d98-9d64-617167da3e6c.jpg)
![캡스톤-최종-ppt-_page-0009](https://user-images.githubusercontent.com/72921481/173407758-291c3a33-d720-4184-b97d-56ee52a46103.jpg)
![캡스톤-최종-ppt-_page-0010](https://user-images.githubusercontent.com/72921481/173407762-736622e1-ac05-46c4-baa9-f67e02202418.jpg)





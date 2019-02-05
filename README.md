# Extended Kalman Filter Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

In this project, I implemented an [Extended Kalman Filter (EKF)](https://en.wikipedia.org/wiki/Extended_Kalman_filter) algorithm in C++ to track and predict a bicycle's position and velocity. I am provided simulated lidar and radar measurements data detecting a bicycle that travels around my vehicle. This Extendted Kalman Filter (EKF) algorithm provides the ability to fusion the measurements from both Lidar and Radar sensors to predict the position and velocity of the bicycle.  The communication between the simulator ([it could be downloaded here](https://github.com/udacity/self-driving-car-sim/releases)) and the EKF is done using [uWebSockets](https://github.com/uNetworking/uWebSockets) implementation on the EKF side. Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles. To valuate the performance of my EKF model, I used the Root Mean Squared Error (RMSE) to accumulate the residual errors between my estimation and the groud truth. My final RMSE on dataset1 is [0.0973, 0.0855, 0.4513, 0.4399], on dataset2 is [0.0726, 0.0965, 0.4216, 0.4932].

## Overview of an Extended Kalman Filter: Initialize, Predict, Update

This project involves three main steps for programming the Extended Kalman Filter (EKF) algorithm:
1. initializing Extended Kalman filter variables
2. predicting where the bicycle is going to be after a time step Δt from the previous time stamp
3. updating where the bicycle is now based on incoming new sensor measurements

Then the prediction and update steps repeat themselves in a loop. To measure how well my Kalman filter performs, I then calculate root mean squared error comparing the Extended Kalman filter results with the provided ground truth. These three steps (initialize, predict, update) plus calculating RMSE encapsulate the entire extended Kalman filter project.The overall fusion flow is illustrated as follows:

![EKF Fusion Flow](https://github.com/zmandyhe/extended-kalman-filter/blob/master/pic/kf-algorithm-fusion-flow.PNG)

# Prerequisites

The project has the following dependencies:

- cmake >= 3.5
- make >= 4.1
- gcc/g++ >= 5.4
- Udacity's simulator.

For instructions on how to install these components on different operating systems, please visit [Udacity's seed project](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project). 

## Compile and Run the Emulator
Clone the repo and cd to it on a Terminal:

1. run '/install-ubuntu.sh` to install wWebSocket, which is open source package that facilitates the connection between the simulator and code in C++. The package does this by setting up a web socket server connection from the C++ program to the simulator, which acts as the host.
2. run 'mkdir build && cd build'
3. run 'cmake ..'
4. run 'make'
5. run './ExtendedKF'
6. Then run the simulator (in my case I used workspace from Udacity, I then click "Simulator" in GPU mode, select "Project 1/2 EKF and UKF"), select "Start" to simulate the prediction and update cycle from the EKF program.

## Description of the Fusion Flow in the Project

### Measurement Data
The code in main.cpp reads in line by line from the data file "obj_pose-laser-radar-synthetic-input.txt" which contains Lidar and Radar data with timestamps. The measurement data for each line gets pushed onto a measurement_pack_list object. Each row contains a radar measurement and ground truth data, and the next row a lidar measurement data and ground truth data, whereas radar has three measurements (rho, phi, rhodot), lidar has two measurements (x, y). The ground truth for each line in the data file gets pushed onto ground_truth so RMSE can be calculated later from tools.cpp.

### Structure of the Source Code

**main.cpp:** - It communicates with the Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE. The involved process is as follows:
* create a FusionEKF instance called fusionEKF
* check if there are new data available, if there is, create a measurement_package object called meas_package to hold the estimation/measurement update data from incoming sensor data and timestamp
* read in groud truth
* call ProcessMeasurement(meas_package) to predict and update the estimation
* call CalculateRMSE() from tools.cpp to calculate RMSE for each position and velocity.

The overall code looks like the following:
```
  FusionEKF fusionEKF;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;
  MeasurementPackage meas_package;
  ground_truth.push_back(gt_values);
  fusionEKF.ProcessMeasurement(meas_package);  
  estimations.push_back(estimate);
  VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
```

**FusionEKF.cpp:** It fusion the flow of prediction and measurement update process, which involves the following several steps:
* initializes the EKF filter parameters, 
* defines the whole ProcessMeasurement() function that includes 
	- Initialize the state vector ekf_.x_
	- read in sensor data (Lidar and Radar respectively) with timestamp, 
	- calls the predict function which updates state transition matrix F [equation is here](https://github.com/zmandyhe/extended-kalman-filter/blob/master/pic/update-F.PNG) and the process covariance matrix Q with  the time elapsed between the current and previous measurements ```"float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; previous_timestamp_ = measurement_pack.timestamp_;"``` before calls the Predic() function from kalman_filter.cpp. Even though Radar prediction function can be non-linear, but in this projet, I used a same linear model to do the prediction for both Lidar and Radar. 
	- after prediction, it calls the update function to perform update according to sensor type. The raw_measurement_ object holds the measurement data of x_ and P_.
	
The prediction function is defined as follows:
```
void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}
```
The update function by sensor type is defined as the following:

```
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
	ekf_.Update(measurement_pack.raw_measurements_);
  }
```
**kalman_filter.cpp:**- It defines the predict function (Predict()), the update function for lidar (Update()), and the update function for radar (UpdateEKF()). The functions are called from FusionEKF.cpp that both Radar and Lidar share the same predict() function, but with different update function. Notice that measurement update is after the prediction, so the update for the state vector x_ and the uncertainty matrix P_ come from the predicted x_ and P_, the equation of calculating them for Lidar and Radar are the same, the key difference is the H matrix that Radar using h(x_) when calculating the error y and using Jacobian Matrix Hj_ when calculating S, K, and P; Lidar uses the initial constant H matrix directly.

**tools.cpp**- function to calculate RMSE and the Jacobian matrix.

### Results
To pass the project, the RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52]. My final RMSE on dataset1 is [0.0973, 0.0855, 0.4513, 0.4399], on dataset2 is [0.0726, 0.0965, 0.4216, 0.4932].

![alt-text-1](https://github.com/zmandyhe/extended-kalman-filter/blob/master/pic/rmse-dataset1.PNG "RMSE on Dataset 1") ![alt-text-2](https://github.com/zmandyhe/extended-kalman-filter/blob/master/pic/rmse-dataset2.PNG "RMSE on Dataset 2")

### Conclusion and Discussion
The project is well alligned with the class discussed about the algorithms of Kalman Filter and Extended Kalman Filter. I spent quite some time to fix this one segmentation fault issue in my Kalman Filter algorithm. This can be avoided easily by simply following the algorithm equation carefully.

To update the state vector x' with the first sensor measurement, for Lidar we need to calculate the error equation is y = z - H * x', then follows the Kalman Filter algorithm to update the state vector. To update for Radar measurements, since  the functions that map the x vector [px, py, vx, vy] to polar coordinates are non-linear, instead of using H to calculate y = z - H * x', for radar measurements I have to use the equations that map from cartesian to polar coordinates: y = z - h(x'), while h(x') falls to this formula [h(x')](https://github.com/zmandyhe/extended-kalman-filter/blob/master/pic/h_x.PNG).

This project provides an opportunity to reflect how the polar coordinates works and the transformation between polar and cartesian coordinates spaces. Radar read in data in ploar coordinates which could be beyond the range of -pi to pi, but the Kalman filter is expecting small angle values between the range -pi and pi.  When calculating phi in y = z - h(x') for radar measurements, the resulting angle phi in the y vector should be adjusted so that it is between -pi and pi. 

This is an interesting project, I could have experimented with one single type of sensor to evaluate the different impacts, and how  does fusing the two sensors' data improve the tracking results. I plan to experiment it in term 2.

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for Matlab scripts that can generate additional data.
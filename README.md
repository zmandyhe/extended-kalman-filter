# Extended Kalman Filter Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

In this project, I implemented an Extended Kalman Filter (EKF) in C++ to track the bicycle's position and velocity. I am provided simulated lidar and radar measurements detecting a bicycle that travels around my vehicle. The Extendted Kalman Filter (EKF) provide the ability to fusion the measurements from both Lidar and Radar to predict the position of the bicyble.  The communication between the simulator and the EKF is done using WebSocket using the uWebSockets implementation on the EKF side. Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles.  To pass the project, the RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52]. final RMSE is [   ].

## Overview of an Extended Kalman Filter: Initialize, Predict, Update
This project involves three main steps for programming a Kalman filter:
* initializing Extended Kalman filter variables
* predicting where our object is going to be after a time step \Delta{t}Δt
* updating where our object is based on sensor measurements
Then the prediction and update steps repeat themselves in a loop.

put the overall flow process here.

To measure how well our Kalman filter performs, we will then calculate root mean squared error comparing the Extended Kalman filter results with the provided ground truth.
These three steps (initialize, predict, update) plus calculating RMSE encapsulate the entire extended Kalman filter project.

##Steps and Details of the Project

### Measurement Data
The code in main.cpp reads in line by line from the data file "obj_pose-laser-radar-synthetic-input.txt". The measurement data for each line gets pushed onto a measurement_pack_list. Each row contains a radar measurement and ground truth data, and the next row a lidar measurement data and ground truth data, whereas radar has three measurements (rho, phi, rhodot), lidar has two measurements (x, y). The ground truth for each line in the data file gets pushed ontoground_truth so RMSE can be calculated later from tools.cpp.

### Structure of the Source Code
The files you need to work with are in the src folder of the github repository.

**main.cpp:** - communicates with the Term 2 Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE. The process is as follows:
* create a FusionEKF instance called fusionEKF
* check if there are new data available, if there is, create a measurement_package object called meas_package to hold the estimation/measurement update data from incoming sensor data and timestamp
* read in groud truth
* call ProcessMeasurement(meas_package) to predict and update the estimation
* call CalculateRMSE() from tools.cpp to calculate RMSE and for each position and velocity.
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
**FusionEKF.cpp:** It involves the following several steps:
* initializes the EKF filter parameters, 
* defines the whole ProcessMeasurement() function that includes 
	- 1) Initialize the state ekf_.x_, 
	- 2) read in sensor data (Lidar and Radar respectively) with timestamp, 
	- 3) calls the predict function whichi involves to update state transition matrix F, update process covariance matrix P_, and using  noise_ax = 9 and noise_ay = 9 for the process covariance matrix Q. Noticed that the prediction function happens when new sensor data comes in, so the time elapsed between the current and previous measurements "  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; previous_timestamp_ = measurement_pack.timestamp_;". dt is used to udpate the F matrix with the following code:
```
	ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;
```
![update F matrix with elapsed time](https://github.com/zmandyhe/extended-kalman-filter/blob/master/pic/update-F.png)
Even though Radar prediction function can be non-linear, but in this projet, I used a linear model to do the prediction for both Lidar and Radar. The prediction function is defined as follows:
```
void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}
```
	- 4) after prediction, we calls the update function to perform update according to sensor type. The raw_measurement_ object holds all the measurement data of x_ and P_.
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
**kalman_filter.cpp:**- defines the predict function (Predict()), the update function for lidar (Update()), and the update function for radar (UpdateEKF()). The functions are called in FusionEKF.cpp that both Radar and Lidar share the same predict() function, but call different update function. Notice that measurement update is after the prediction, so the update for the state x_ and the uncertainty matrix P_ come from the predicted x_ and P_, the equation of calculating them for Lidar and Radar are the same, the key difference is the H matrix that Radar using h(x_) and Jacobian Matrix Hj_, and Lidar uses the initial H matrix directly.
* tools.cpp- function to calculate RMSE and the Jacobian matrix

**Convert Radar input data from polar coordinates to Cartesian coordinates**

The Radar measurement data are:
z = ()

convert the polar measurements into cartesian coordinate system.

x = cos(phi) * rho
y = sin(phi) * rho
vx = cos(phi) * d-rho
vy = sin(phi) * d-rho


##Compile and Run the Emulator
Once the code is ready to compile, from the project folder:

1. run '/install-ubuntu.sh` to install wWebSocket, which is open source package that facilitates the connection between the simulator and code in C++. The package does this by setting up a web socket server connection from the C++ program to the simulator, which acts as the host.
2. run 'mkdir build && cd build'
3. run 'cmake ..'
4. run 'make'
5. run './ExtendedKF'

Then click "Simulator", select "Project 1/2 EKF and UKF", select "Start" to simulate the prediction and update cycle from the EKF program.

### Results
. 

### Following the Correct Algorithm


### Conclusion and Discussion


## Reproduce This Project
Follow below process to reproduce the training and prediction process:
* If you don't have the lab environment, you can download and install it from [CarND Term1 Starter Kit](https://github.com/udacity/CarND-Term1-Starter-Kit)
* The simulator can be downloaded from GitHub [Udacity Simulator](https://github.com/udacity/self-driving-car-sim). You can use the "training mode" to collect training datasets directly from the sitmulator, which produces video frame images and driving_log.csv with all the 3 cameras view images and steering angle data.
* To train your model, run the "python3 model.py"
* To test on autonomous mode, run "python 3 drive.py model.h5 run1"
* To save the autonomous run from "run1" to video, run"python video.py run1 --fps 48"

## Reproduce the project
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.
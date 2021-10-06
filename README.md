[![UnitTests](https://github.com/borodziejciesla/bayesian_filters/actions/workflows/UnitTests.yml/badge.svg)](https://github.com/borodziejciesla/bayesian_filters/actions/workflows/UnitTests.yml)
[![Build Status](https://app.travis-ci.com/borodziejciesla/bayesian_filters.svg?token=xzbGQFuWuWjAwFgp8FyQ&branch=main)](https://app.travis-ci.com/borodziejciesla/bayesian_filters)
[![codecov](https://codecov.io/gh/borodziejciesla/bayesian_filters/branch/main/graph/badge.svg?token=4Q0b0fu6U7)](https://codecov.io/gh/borodziejciesla/bayesian_filters)

# Bayesian Filters

***
## Overview
C++ implementation of different types of Bayesian filters
***
## Algorithms
---
### Kalman Filter
#### Process Model
Prediction model:

<a href="https://www.codecogs.com/eqnedit.php?latex=x_{k}&space;=&space;A_{k}x_{k-1}&space;&plus;&space;B_{k}u_{k}&space;&plus;&space;G_{k}w_{k}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?x_{k}&space;=&space;A_{k}x_{k-1}&space;&plus;&space;B_{k}u_{k}&space;&plus;&space;G_{k}w_{k}" title="x_{k} = A_{k}x_{k-1} + B_{k}u_{k} + G_{k}w_{k}" /></a>

Where:
* $w_{k}$ - Gaussian noise $\mathit{N}(0, Q_{k})$

Measurement model:
$$
  y_{k} = C_{k}x_{k} + v_{k}
$$
Where:
* $v_{k}$ - Gaussian noise $\mathit{N}(0, R_{k})$

#### Prediction Step
$$
  \begin{array}{l}
    x_{k|k-1} = A_{k}x_{k-1|k-1} + B_{k}u_{k} + G_{k}w_{k}\\
    P_{k|k-1} = A_{k}P_{k|k}A_{k}^{T} + B_{k}\Sigma_{k}B_{k}^{T} + G_{k}Q_{k}G_{k}^{T}
  \end{array}
$$

#### Correction Step
$$  
  \begin{array}{l}
    z_{k} = y_{k} - C_{k}x_{k|k-1}\\
    S_{k} = C_{k}P_{k|k-1}C_{k}^{T} + R_{k}\\
    K_{k} = P_{k|k-1}C_{k}^{T}S_{k}^{-1}\\
    \\
    x_{k|k} = x_{k|k-1} + K_{k}z_{k}\\
    P_{k|k} = (I - K_{k}C_{k})P_{k|k-1}
  \end{array}
$$

---
### Extended Kalman Filter
#### Process Model
Prediction model:
$$
  x_{k} = f_{k}(x_{k-1}, u_{k}) + G_{k}w_{k}
$$
Where:
* $w_{k}$ - Gaussian noise $\mathit{N}(0, Q_{k})$

Measurement model:
$$
  y_{k} = g_{k}(x_{k}) + v_{k}
$$
Where:
* $v_{k}$ - Gaussian noise $\mathit{N}(0, R_{k})$

#### Prediction Step
$$
  \begin{array}{l}
    x_{k|k-1} = f_{k}(x_{k-1|k-1}, u_{k}) + G_{k}w_{k}\\
    P_{k|k-1} = AP_{k|k}A^{T} + B\Sigma_{k}B^{T} + G_{k}Q_{k}G_{k}^{T}
  \end{array}
$$
Where:
* $A = \frac{\partial}{\partial x}f_{k}(x, u)|_{x=x_{k|k-1}, u=u_{k}}$,
* $B = \frac{\partial}{\partial x}f_{k}(x, u)|_{x=x_{k|k-1}, u=u_{k}}$.

#### Correction Step
$$  
  \begin{array}{l}
    z_{k} = y_{k} - g_{k}(x_{k|k-1})\\
    S_{k} = CP_{k|k-1}C^{T} + R_{k}\\
    K_{k} = P_{k|k-1}C^{T}S_{k}^{-1}\\
    \\
    x_{k|k} = x_{k|k-1} + K_{k}z_{k}\\
    P_{k|k} = (I - K_{k}C)P_{k|k-1}
  \end{array}
$$
Where:
* $C = \frac{\partial}{\partial}g_{k}(x)|_{x=x_{k|k-1}}$

---
### Unscented Kalman Filter
#### Process Model
$$
  x_{k} = f_{k}(x_{k-1}, u_{k}) + G_{k}w_{k}
$$
Where:
* $w_{k}$ - Gaussian noise $\mathit{N}(0, Q_{k})$

Measurement model:
$$
  y_{k} = g_{k}(x_{k}) + v_{k}
$$
Where:
* $v_{k}$ - Gaussian noise $\mathit{N}(0, R_{k})$

#### Selection of Sigma Points
$$
  \chi = 
$$
#### Prediction Step
$$
  \begin{array}{l}
    x_{k|k-1} = \Sigma_{i=0}^{N}{w_{i}f_{k}(x_{k|k-1}, u_{k})}\\
    P_{k|k-1} = \Sigma_{i=1}^{N}{w_{i}(f_{k}(\chi_{i}) - x_{k|k-1})((f_{k}(\chi_{i}) - x_{k|k-1})^{T}} + G_{k}Q_{k}G_{k}^{T}
  \end{array}
$$

#### Correction Step
Predicted measureinnovationment with covariance:
$$
  \begin{array}{l}
    \hat{y}_{k} = \Sigma_{i=0}^{N}{w_{i}g_{k}(\chi_{i})}\\
    S_{k} = \Sigma_{i=0}^{N}{w_{i}\left(g_{k}(\chi_{i}) - y_{k}\right)(g_{k}\left(\chi_{i}) - y_{k}\right)^{T}} + R_{k}
  \end{array}
$$
Kalman gain calculation::
$$
  \begin{array}{l}
    T_{k} = \Sigma_{i=0}^{N}{w_{i}(\chi_{i} - x_{k|k-1})(g_{k}(\chi_{i}) - y_{k})^{T}}\\
    K_{k} = T_{k}S_{k}^{-1}
  \end{array}
$$
Calculate estimation:
$$
  \begin{array}{l}
    x_{k|k} = x_{k|k-1} + K_{k}(y_{k} - \hat{y}_{k})\\
    P_{k|k} = (I - K_{k}T_{k})P_{k|k-1}
  \end{array}
$$

***
## Usage
### Prepare calibration
Before we create filter  we need to create calibration. Calibrations are presented by strusture `bf_io::FilterCalibration`, where:
* `state_dimension` - dimension of state vector $x$,
* `measurement_dimension` - dimension of measurement vector $y$,
* `transition` - function object with process prediction function $f(x,u)$,
* `transition_jacobian` - function object with observation jacobian $\frac{\partial}{\partial x}g(x)$,
* `observation` - function object with process prediction function $\frac{\partial}{\partial x}f(x,u)$,
* `observation_jacobian` - function object with observation jacobian $\frac{\partial}{\partial x}g(x)$,
* `proccess_noise_covariance` - process noise covariance, this structure contains matrix diagonal and lower triangle of matrix.

For exmple in case of simple process:
$$
  \begin{array}{l}
    x_{k}
    =
    \left[
    \begin{array}{c}
      x_{k}^{1}\\
      x_{k}^{2}
    \end{array}
    \right]
    =
    \left[
    \begin{array}{c}
      x_{k-1}^{1} + Tx_{k-1}^{2}\\
      \cos{x_{k-1}^{2}} + u
    \end{array}
    \right]
    +
    \left[
    \begin{array}{cc}
      1 & 0\\
      0 & 1
    \end{array}
    \right]
    \left[
    \begin{array}{c}
      w_{k}^{1}\\
      w_{k}^{2}
    \end{array}
    \right]\\
    \\
    y_{k}
    =
    x_{k}^{1}
  \end{array}
$$
So we have:
* $$f(x,u)
  =
  \left[
    \begin{array}{c}
      x^{1} + Tx^{2}\\
      \cos{x^{2}} + u
    \end{array}
  \right]
  $$
* $$
    \frac{\partial}{\partial x}f(x,u)
    =
    \left[
    \begin{array}{cc}
      1 & T\\
      0 & \sin{(x^{2})}
    \end{array}
    \right]
  $$
* $$g(x) = x^{1}$$
* $$
    \frac{\partial}{\partial x}g(x)
    =
    \left[
    \begin{array}{cc}
      1 & 0
    \end{array}
    \right]
  $$
Based on that calculations we can set filter calibrations:
```cpp
bf_io::FilterCalibration calibration;
calibration.state_dimension = 2u;
calibration.measurement_dimension = 1u;

calibration.transition = [](const Eigen::VectorXf & state, const float time_delta) {
  Eigen::VectorXf transformed_state;
  transformed_state(0) = state(0) + time_delta * state(1);
  transformed_state(1) = std::cos(state(1));

  return transformed_state;
};

calibration.transition_jacobian = [](const Eigen::VectorXf & state, const float time_delta) {
  Eigen::MatrixXf jacobian;
  jacobian(0, 0) = 1.0f;
  jacobian(0, 1) = time_delta;
  jacobian(1, 0) = 0.0f;
  jacobian(1, 1) = std::sin(state(1));

  return jacobian;
};

calibration.observation = [](const Eigen::VectorXf & state) {
  Eigen::VectorXf observation(1, 1);
  jacobian(0, 0) = state(0);

  return observation;
};

calibration.observation_jacobian = [](const Eigen::VectorXf & state) {
  Eigen::MatrixXf jacobian(1, 2);
  jacobian(0, 0) = 1.0f;
  jacobian(0, 1) = 0.0f;

  return jacobian;
};

calibration.state_dimension = 2u;
```

### Create filter
For creating filter we need to use previously prepared calibration and set filter type. For example for creating Extended Kalman Filter following code can be used:
```cpp
bf_io::FilterType filter_type = bf_io::FilterType::KF;
bf::BayesianFilter filter(filter_type, calibration);
```

### Meaurement input format
Measurements are provided witj structure *bf_io::ValueWithTimestampAndCovariance*. This structure contains timestamp, measurement value and measurement covariance (covariance structure contains diagonal and lower/upper triangle of matrix).

### Run filter
New measurement is processed by filter by calling method:
```cpp
void RunFilter(const bf_io::ValueWithTimestampAndCovariance & measurement);
```
This methods run prediction and correction step. Current estimation of state is read by method:
```cpp
const bf_io::ValueWithTimestampAndCovariance & GetEstimation(void);
```
Result is in the same format as measurement, contains timestamp of estimation, estimated value and estimation covariance.

***
## Development environment
* Programming language:
  * C++20 standard - g++10.3 or higher
* Build system:
  * CMake 3.12 or higher
* Static code analysis:
  * cppcheck 1.90 or higher
  * pygments
* Test coverage:
  * gcov
  * lcov
* Coding style check
  * cpplint 1.5.5 or higher

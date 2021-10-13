[![UnitTests](https://github.com/borodziejciesla/bayesian_filters/actions/workflows/UnitTests.yml/badge.svg)](https://github.com/borodziejciesla/bayesian_filters/actions/workflows/UnitTests.yml)
[![Build Status](https://app.travis-ci.com/borodziejciesla/bayesian_filters.svg?token=xzbGQFuWuWjAwFgp8FyQ&branch=main)](https://app.travis-ci.com/borodziejciesla/bayesian_filters)
[![codecov](https://codecov.io/gh/borodziejciesla/bayesian_filters/branch/main/graph/badge.svg?token=4Q0b0fu6U7)](https://codecov.io/gh/borodziejciesla/bayesian_filters)

# Bayesian Filters

***
## Overview
C++ implementation of different types of Bayesian filters:
* Linear Kalman Filter
* Extended Kalman Filter
* Unscented Kalman Filter
***
### How to build?
* Simple build:

```
cmake -S . -B build
cmake --build build
```
* Running static code analysis with *cppcheck*, report is located in *build/cppcheck_output_html/index.html*.:
```
cmake -S . -B build -DMAKE_CPPCHECK=ON
cmake --build build
cd build
make cppcheck       # Runs static code analysis
make cppcheck_html  # Creates HTML report
```
* Running code style analysis with *cpplint*:
```
cmake -S . -B build -DMAKE_CPPLINT=ON
cmake --build build
cd build
make cpplint       # Runs code style analysis
```
* Running test coverage, report is located in *build/bayesian_filters_coverage_report/index.html*:
```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Coverage
cmake --build build
cd build
make coverage       # Runs code style analysis
```
* Build with example (*build/examples/bayesian_filters_example*):
```
cmake -S . -B build -DBUILD_EXAMPLE=ON
cmake --build build
```
Run this example in following way:
```
bayesian_filters_example number_of_samples
```

## Algorithms
---
### Kalman Filter
#### Process Model
Prediction model:

<a href="https://www.codecogs.com/eqnedit.php?latex=x_{k}&space;=&space;A_{k}x_{k-1}&space;&plus;&space;B_{k}u_{k}&space;&plus;&space;G_{k}w_{k}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?x_{k}&space;=&space;A_{k}x_{k-1}&space;&plus;&space;B_{k}u_{k}&space;&plus;&space;G_{k}w_{k}" title="x_{k} = A_{k}x_{k-1} + B_{k}u_{k} + G_{k}w_{k}" /></a>

Where:
* <a href="https://www.codecogs.com/eqnedit.php?latex=w_{k}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?w_{k}" title="w_{k}" /></a> - Gaussian noise <a href="https://www.codecogs.com/eqnedit.php?latex=\mathit{N}(0,&space;Q_{k})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathit{N}(0,&space;Q_{k})" title="\mathit{N}(0, Q_{k})" /></a>.

Measurement model:

<a href="https://www.codecogs.com/eqnedit.php?latex=y_{k}&space;=&space;C_{k}x_{k}&space;&plus;&space;v_{k}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?y_{k}&space;=&space;C_{k}x_{k}&space;&plus;&space;v_{k}" title="y_{k} = C_{k}x_{k} + v_{k}" /></a>

Where:
* <a href="https://www.codecogs.com/eqnedit.php?latex=v_{k}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v_{k}" title="v_{k}" /></a> - Gaussian noise <a href="https://www.codecogs.com/eqnedit.php?latex=\mathit{N}(0,&space;R_{k})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathit{N}(0,&space;R_{k})" title="\mathit{N}(0, R_{k})" /></a>

#### Prediction Step
<a href="https://www.codecogs.com/eqnedit.php?latex=\begin{array}{l}&space;x_{k|k-1}&space;=&space;A_{k}x_{k-1|k-1}&space;&plus;&space;B_{k}u_{k}&space;&plus;&space;G_{k}w_{k}\\&space;P_{k|k-1}&space;=&space;A_{k}P_{k|k}A_{k}^{T}&space;&plus;&space;B_{k}\Sigma_{k}B_{k}^{T}&space;&plus;&space;G_{k}Q_{k}G_{k}^{T}&space;\end{array}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\begin{array}{l}&space;x_{k|k-1}&space;=&space;A_{k}x_{k-1|k-1}&space;&plus;&space;B_{k}u_{k}&space;&plus;&space;G_{k}w_{k}\\&space;P_{k|k-1}&space;=&space;A_{k}P_{k|k}A_{k}^{T}&space;&plus;&space;B_{k}\Sigma_{k}B_{k}^{T}&space;&plus;&space;G_{k}Q_{k}G_{k}^{T}&space;\end{array}" title="\begin{array}{l} x_{k|k-1} = A_{k}x_{k-1|k-1} + B_{k}u_{k} + G_{k}w_{k}\\ P_{k|k-1} = A_{k}P_{k|k}A_{k}^{T} + B_{k}\Sigma_{k}B_{k}^{T} + G_{k}Q_{k}G_{k}^{T} \end{array}" /></a>

#### Correction Step
<a href="https://www.codecogs.com/eqnedit.php?latex=\begin{array}{l}&space;z_{k}&space;=&space;y_{k}&space;-&space;C_{k}x_{k|k-1}\\&space;S_{k}&space;=&space;C_{k}P_{k|k-1}C_{k}^{T}&space;&plus;&space;R_{k}\\&space;K_{k}&space;=&space;P_{k|k-1}C_{k}^{T}S_{k}^{-1}\\&space;\\&space;x_{k|k}&space;=&space;x_{k|k-1}&space;&plus;&space;K_{k}z_{k}\\&space;P_{k|k}&space;=&space;(I&space;-&space;K_{k}C_{k})P_{k|k-1}&space;\end{array}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\begin{array}{l}&space;z_{k}&space;=&space;y_{k}&space;-&space;C_{k}x_{k|k-1}\\&space;S_{k}&space;=&space;C_{k}P_{k|k-1}C_{k}^{T}&space;&plus;&space;R_{k}\\&space;K_{k}&space;=&space;P_{k|k-1}C_{k}^{T}S_{k}^{-1}\\&space;\\&space;x_{k|k}&space;=&space;x_{k|k-1}&space;&plus;&space;K_{k}z_{k}\\&space;P_{k|k}&space;=&space;(I&space;-&space;K_{k}C_{k})P_{k|k-1}&space;\end{array}" title="\begin{array}{l} z_{k} = y_{k} - C_{k}x_{k|k-1}\\ S_{k} = C_{k}P_{k|k-1}C_{k}^{T} + R_{k}\\ K_{k} = P_{k|k-1}C_{k}^{T}S_{k}^{-1}\\ \\ x_{k|k} = x_{k|k-1} + K_{k}z_{k}\\ P_{k|k} = (I - K_{k}C_{k})P_{k|k-1} \end{array}" /></a>

---
### Extended Kalman Filter
#### Process Model
Prediction model:

<a href="https://www.codecogs.com/eqnedit.php?latex=x_{k}&space;=&space;f_{k}(x_{k-1},&space;u_{k})&space;&plus;&space;G_{k}w_{k}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?x_{k}&space;=&space;f_{k}(x_{k-1},&space;u_{k})&space;&plus;&space;G_{k}w_{k}" title="x_{k} = f_{k}(x_{k-1}, u_{k}) + G_{k}w_{k}" /></a>

Where:
* <a href="https://www.codecogs.com/eqnedit.php?latex=w_{k}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?w_{k}" title="w_{k}" /></a> - Gaussian noise <a href="https://www.codecogs.com/eqnedit.php?latex=mathit{N}(0,&space;Q_{k})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathit{N}(0,&space;Q_{k})" title="\mathit{N}(0, Q_{k})" /></a>

Measurement model:

<a href="https://www.codecogs.com/eqnedit.php?latex=y_{k}&space;=&space;g_{k}(x_{k})&space;&plus;&space;v_{k}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?y_{k}&space;=&space;g_{k}(x_{k})&space;&plus;&space;v_{k}" title="y_{k} = g_{k}(x_{k}) + v_{k}" /></a>

Where:
* <a href="https://www.codecogs.com/eqnedit.php?latex=v_{k}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v_{k}" title="v_{k}" /></a> - Gaussian noise <a href="https://www.codecogs.com/eqnedit.php?latex=\mathit{N}(0,&space;R_{k})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathit{N}(0,&space;R_{k})" title="\mathit{N}(0, R_{k})" /></a>

#### Prediction Step
<a href="https://www.codecogs.com/eqnedit.php?latex=\begin{array}{l}&space;x_{k|k-1}&space;=&space;f_{k}(x_{k-1|k-1},&space;u_{k})&space;&plus;&space;G_{k}w_{k}\\&space;P_{k|k-1}&space;=&space;AP_{k|k}A^{T}&space;&plus;&space;B\Sigma_{k}B^{T}&space;&plus;&space;G_{k}Q_{k}G_{k}^{T}&space;\end{array}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\begin{array}{l}&space;x_{k|k-1}&space;=&space;f_{k}(x_{k-1|k-1},&space;u_{k})&space;&plus;&space;G_{k}w_{k}\\&space;P_{k|k-1}&space;=&space;AP_{k|k}A^{T}&space;&plus;&space;B\Sigma_{k}B^{T}&space;&plus;&space;G_{k}Q_{k}G_{k}^{T}&space;\end{array}" title="\begin{array}{l} x_{k|k-1} = f_{k}(x_{k-1|k-1}, u_{k}) + G_{k}w_{k}\\ P_{k|k-1} = AP_{k|k}A^{T} + B\Sigma_{k}B^{T} + G_{k}Q_{k}G_{k}^{T} \end{array}" /></a>

Where:
* <a href="https://www.codecogs.com/eqnedit.php?latex=A&space;=&space;\frac{\partial}{\partial&space;x}f_{k}(x,&space;u)|_{x=x_{k|k-1},&space;u=u_{k}}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?A&space;=&space;\frac{\partial}{\partial&space;x}f_{k}(x,&space;u)|_{x=x_{k|k-1},&space;u=u_{k}}" title="A = \frac{\partial}{\partial x}f_{k}(x, u)|_{x=x_{k|k-1}, u=u_{k}}" /></a>,
* <a href="https://www.codecogs.com/eqnedit.php?latex=B&space;=&space;\frac{\partial}{\partial&space;x}f_{k}(x,&space;u)|_{x=x_{k|k-1},&space;u=u_{k}}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?B&space;=&space;\frac{\partial}{\partial&space;x}f_{k}(x,&space;u)|_{x=x_{k|k-1},&space;u=u_{k}}" title="B = \frac{\partial}{\partial x}f_{k}(x, u)|_{x=x_{k|k-1}, u=u_{k}}" /></a>.

#### Correction Step
<a href="https://www.codecogs.com/eqnedit.php?latex=\begin{array}{l}&space;z_{k}&space;=&space;y_{k}&space;-&space;g_{k}(x_{k|k-1})\\&space;S_{k}&space;=&space;CP_{k|k-1}C^{T}&space;&plus;&space;R_{k}\\&space;K_{k}&space;=&space;P_{k|k-1}C^{T}S_{k}^{-1}\\&space;\\&space;x_{k|k}&space;=&space;x_{k|k-1}&space;&plus;&space;K_{k}z_{k}\\&space;P_{k|k}&space;=&space;(I&space;-&space;K_{k}C)P_{k|k-1}&space;\end{array}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\begin{array}{l}&space;z_{k}&space;=&space;y_{k}&space;-&space;g_{k}(x_{k|k-1})\\&space;S_{k}&space;=&space;CP_{k|k-1}C^{T}&space;&plus;&space;R_{k}\\&space;K_{k}&space;=&space;P_{k|k-1}C^{T}S_{k}^{-1}\\&space;\\&space;x_{k|k}&space;=&space;x_{k|k-1}&space;&plus;&space;K_{k}z_{k}\\&space;P_{k|k}&space;=&space;(I&space;-&space;K_{k}C)P_{k|k-1}&space;\end{array}" title="\begin{array}{l} z_{k} = y_{k} - g_{k}(x_{k|k-1})\\ S_{k} = CP_{k|k-1}C^{T} + R_{k}\\ K_{k} = P_{k|k-1}C^{T}S_{k}^{-1}\\ \\ x_{k|k} = x_{k|k-1} + K_{k}z_{k}\\ P_{k|k} = (I - K_{k}C)P_{k|k-1} \end{array}" /></a>

Where:
* <a href="https://www.codecogs.com/eqnedit.php?latex=C&space;=&space;\frac{\partial}{\partial}g_{k}(x)|_{x=x_{k|k-1}}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?C&space;=&space;\frac{\partial}{\partial}g_{k}(x)|_{x=x_{k|k-1}}" title="C = \frac{\partial}{\partial}g_{k}(x)|_{x=x_{k|k-1}}" /></a>

---
### Unscented Kalman Filter
#### Process Model
<a href="https://www.codecogs.com/eqnedit.php?latex=x_{k}&space;=&space;f_{k}(x_{k-1},&space;u_{k})&space;&plus;&space;G_{k}w_{k}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?x_{k}&space;=&space;f_{k}(x_{k-1},&space;u_{k})&space;&plus;&space;G_{k}w_{k}" title="x_{k} = f_{k}(x_{k-1}, u_{k}) + G_{k}w_{k}" /></a>
  
Where:
* <a href="https://www.codecogs.com/eqnedit.php?latex=w_{k}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?w_{k}" title="w_{k}" /></a> - Gaussian noise <a href="https://www.codecogs.com/eqnedit.php?latex=\mathit{N}(0,&space;Q_{k})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathit{N}(0,&space;Q_{k})" title="\mathit{N}(0, Q_{k})" /></a>

Measurement model:

<a href="https://www.codecogs.com/eqnedit.php?latex=y_{k}&space;=&space;g_{k}(x_{k})&space;&plus;&space;v_{k}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?y_{k}&space;=&space;g_{k}(x_{k})&space;&plus;&space;v_{k}" title="y_{k} = g_{k}(x_{k}) + v_{k}" /></a>

Where:
* <a href="https://www.codecogs.com/eqnedit.php?latex=v_{k}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v_{k}" title="v_{k}" /></a> - Gaussian noise <a href="https://www.codecogs.com/eqnedit.php?latex=\mathit{N}(0,&space;R_{k})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathit{N}(0,&space;R_{k})" title="\mathit{N}(0, R_{k})" /></a>

#### Selection of Sigma Points
<a href="https://www.codecogs.com/eqnedit.php?latex=\chi_{k}&space;=&space;\left&space;[&space;x_{k-1|k-1},&space;x_{k-1|k-1}&space;\pm&space;\sqrt{(L&plus;\lambda)P_{k-1|k-1}}&space;\right&space;]" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\chi_{k}&space;=&space;\left&space;[&space;x_{k-1|k-1},&space;x_{k-1|k-1}&space;\pm&space;\sqrt{(L&plus;\lambda)P_{k-1|k-1}}&space;\right&space;]" title="\chi_{k} = \left [ x_{k-1|k-1}, x_{k-1|k-1} \pm \sqrt{(L+\lambda)P_{k-1|k-1}} \right ]" /></a>
#### Prediction Step
<a href="https://www.codecogs.com/eqnedit.php?latex=\begin{array}{l}&space;x_{k|k-1}&space;=&space;\Sigma_{i=0}^{N}{w_{i}f_{k}(x_{k|k-1},&space;u_{k})}\\&space;P_{k|k-1}&space;=&space;\Sigma_{i=1}^{N}{w_{i}(f_{k}(\chi_{i})&space;-&space;x_{k|k-1})((f_{k}(\chi_{i})&space;-&space;x_{k|k-1})^{T}}&space;&plus;&space;G_{k}Q_{k}G_{k}^{T}&space;\end{array}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\begin{array}{l}&space;x_{k|k-1}&space;=&space;\Sigma_{i=0}^{N}{w_{i}f_{k}(x_{k|k-1},&space;u_{k})}\\&space;P_{k|k-1}&space;=&space;\Sigma_{i=1}^{N}{w_{i}(f_{k}(\chi_{i})&space;-&space;x_{k|k-1})((f_{k}(\chi_{i})&space;-&space;x_{k|k-1})^{T}}&space;&plus;&space;G_{k}Q_{k}G_{k}^{T}&space;\end{array}" title="\begin{array}{l} x_{k|k-1} = \Sigma_{i=0}^{N}{w_{i}f_{k}(x_{k|k-1}, u_{k})}\\ P_{k|k-1} = \Sigma_{i=1}^{N}{w_{i}(f_{k}(\chi_{i}) - x_{k|k-1})((f_{k}(\chi_{i}) - x_{k|k-1})^{T}} + G_{k}Q_{k}G_{k}^{T} \end{array}" /></a>

#### Correction Step
Predicted measureinnovationment with covariance:

<a href="https://www.codecogs.com/eqnedit.php?latex=\begin{array}{l}&space;\hat{y}_{k}&space;=&space;\Sigma_{i=0}^{N}{w_{i}g_{k}(\chi_{i})}\\&space;S_{k}&space;=&space;\Sigma_{i=0}^{N}{w_{i}\left(g_{k}(\chi_{i})&space;-&space;y_{k}\right)(g_{k}\left(\chi_{i})&space;-&space;y_{k}\right)^{T}}&space;&plus;&space;R_{k}&space;\end{array}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\begin{array}{l}&space;\hat{y}_{k}&space;=&space;\Sigma_{i=0}^{N}{w_{i}g_{k}(\chi_{i})}\\&space;S_{k}&space;=&space;\Sigma_{i=0}^{N}{w_{i}\left(g_{k}(\chi_{i})&space;-&space;y_{k}\right)(g_{k}\left(\chi_{i})&space;-&space;y_{k}\right)^{T}}&space;&plus;&space;R_{k}&space;\end{array}" title="\begin{array}{l} \hat{y}_{k} = \Sigma_{i=0}^{N}{w_{i}g_{k}(\chi_{i})}\\ S_{k} = \Sigma_{i=0}^{N}{w_{i}\left(g_{k}(\chi_{i}) - y_{k}\right)(g_{k}\left(\chi_{i}) - y_{k}\right)^{T}} + R_{k} \end{array}" /></a>

Kalman gain calculation:

<a href="https://www.codecogs.com/eqnedit.php?latex=\begin{array}{l}&space;T_{k}&space;=&space;\Sigma_{i=0}^{N}{w_{i}(\chi_{i}&space;-&space;x_{k|k-1})(g_{k}(\chi_{i})&space;-&space;y_{k})^{T}}\\&space;K_{k}&space;=&space;T_{k}S_{k}^{-1}&space;\end{array}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\begin{array}{l}&space;T_{k}&space;=&space;\Sigma_{i=0}^{N}{w_{i}(\chi_{i}&space;-&space;x_{k|k-1})(g_{k}(\chi_{i})&space;-&space;y_{k})^{T}}\\&space;K_{k}&space;=&space;T_{k}S_{k}^{-1}&space;\end{array}" title="\begin{array}{l} T_{k} = \Sigma_{i=0}^{N}{w_{i}(\chi_{i} - x_{k|k-1})(g_{k}(\chi_{i}) - y_{k})^{T}}\\ K_{k} = T_{k}S_{k}^{-1} \end{array}" /></a>

Calculate estimation:

<a href="https://www.codecogs.com/eqnedit.php?latex=\begin{array}{l}&space;x_{k|k}&space;=&space;x_{k|k-1}&space;&plus;&space;K_{k}(y_{k}&space;-&space;\hat{y}_{k})\\&space;P_{k|k}&space;=&space;(I&space;-&space;K_{k}T_{k})P_{k|k-1}&space;\end{array}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\begin{array}{l}&space;x_{k|k}&space;=&space;x_{k|k-1}&space;&plus;&space;K_{k}(y_{k}&space;-&space;\hat{y}_{k})\\&space;P_{k|k}&space;=&space;(I&space;-&space;K_{k}T_{k})P_{k|k-1}&space;\end{array}" title="\begin{array}{l} x_{k|k} = x_{k|k-1} + K_{k}(y_{k} - \hat{y}_{k})\\ P_{k|k} = (I - K_{k}T_{k})P_{k|k-1} \end{array}" /></a>

***
## Usage
### Prepare calibration
Before we create filter  we need to create calibration. Calibrations are presented by strusture `bf_io::FilterCalibration`, where:
* `state_dimension` - dimension of state vector <a href="https://www.codecogs.com/eqnedit.php?latex=x" target="_blank"><img src="https://latex.codecogs.com/gif.latex?x" title="x" /></a>,
* `measurement_dimension` - dimension of measurement vector <a href="https://www.codecogs.com/eqnedit.php?latex=y" target="_blank"><img src="https://latex.codecogs.com/gif.latex?y" title="y" /></a>,
* `transition` - function object with process prediction function <a href="https://www.codecogs.com/eqnedit.php?latex=f(x,u)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?f(x,u)" title="f(x,u)" /></a>,
* `transition_jacobian` - function object with observation jacobian <a href="https://www.codecogs.com/eqnedit.php?latex=\frac{\partial}{\partial&space;x}g(x)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\frac{\partial}{\partial&space;x}g(x)" title="\frac{\partial}{\partial x}g(x)" /></a>,
* `observation` - function object with process prediction function <a href="https://www.codecogs.com/eqnedit.php?latex=\frac{\partial}{\partial&space;x}f(x,u)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\frac{\partial}{\partial&space;x}f(x,u)" title="\frac{\partial}{\partial x}f(x,u)" /></a>,
* `observation_jacobian` - function object with observation jacobian <a href="https://www.codecogs.com/eqnedit.php?latex=\frac{\partial}{\partial&space;x}g(x)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\frac{\partial}{\partial&space;x}g(x)" title="\frac{\partial}{\partial x}g(x)" /></a>,
* `proccess_noise_covariance` - process noise covariance, this structure contains matrix diagonal and lower triangle of matrix.

For exmple in case of simple process:

<a href="https://www.codecogs.com/eqnedit.php?latex=\begin{array}{l}&space;x_{k}&space;=&space;\left[&space;\begin{array}{c}&space;x_{k}^{1}\\&space;x_{k}^{2}&space;\end{array}&space;\right]&space;=&space;\left[&space;\begin{array}{c}&space;x_{k-1}^{1}&space;&plus;&space;Tx_{k-1}^{2}\\&space;\cos{x_{k-1}^{2}}&space;&plus;&space;u&space;\end{array}&space;\right]&space;&plus;&space;\left[&space;\begin{array}{cc}&space;1&space;&&space;0\\&space;0&space;&&space;1&space;\end{array}&space;\right]&space;\left[&space;\begin{array}{c}&space;w_{k}^{1}\\&space;w_{k}^{2}&space;\end{array}&space;\right]\\&space;\\&space;y_{k}&space;=&space;x_{k}^{1}&space;\end{array}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\begin{array}{l}&space;x_{k}&space;=&space;\left[&space;\begin{array}{c}&space;x_{k}^{1}\\&space;x_{k}^{2}&space;\end{array}&space;\right]&space;=&space;\left[&space;\begin{array}{c}&space;x_{k-1}^{1}&space;&plus;&space;Tx_{k-1}^{2}\\&space;\cos{x_{k-1}^{2}}&space;&plus;&space;u&space;\end{array}&space;\right]&space;&plus;&space;\left[&space;\begin{array}{cc}&space;1&space;&&space;0\\&space;0&space;&&space;1&space;\end{array}&space;\right]&space;\left[&space;\begin{array}{c}&space;w_{k}^{1}\\&space;w_{k}^{2}&space;\end{array}&space;\right]\\&space;\\&space;y_{k}&space;=&space;x_{k}^{1}&space;\end{array}" title="\begin{array}{l} x_{k} = \left[ \begin{array}{c} x_{k}^{1}\\ x_{k}^{2} \end{array} \right] = \left[ \begin{array}{c} x_{k-1}^{1} + Tx_{k-1}^{2}\\ \cos{x_{k-1}^{2}} + u \end{array} \right] + \left[ \begin{array}{cc} 1 & 0\\ 0 & 1 \end{array} \right] \left[ \begin{array}{c} w_{k}^{1}\\ w_{k}^{2} \end{array} \right]\\ \\ y_{k} = x_{k}^{1} \end{array}" /></a>

So we have:
* <a href="https://www.codecogs.com/eqnedit.php?latex=f(x,u)&space;=&space;\left[&space;\begin{array}{c}&space;x^{1}&space;&plus;&space;Tx^{2}\\&space;\cos{x^{2}}&space;&plus;&space;u&space;\end{array}&space;\right]" target="_blank"><img src="https://latex.codecogs.com/gif.latex?f(x,u)&space;=&space;\left[&space;\begin{array}{c}&space;x^{1}&space;&plus;&space;Tx^{2}\\&space;\cos{x^{2}}&space;&plus;&space;u&space;\end{array}&space;\right]" title="f(x,u) = \left[ \begin{array}{c} x^{1} + Tx^{2}\\ \cos{x^{2}} + u \end{array} \right]" /></a>
* <a href="https://www.codecogs.com/eqnedit.php?latex=\frac{\partial}{\partial&space;x}f(x,u)&space;=&space;\left[&space;\begin{array}{cc}&space;1&space;&&space;T\\&space;0&space;&&space;\sin{(x^{2})}&space;\end{array}&space;\right]" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\frac{\partial}{\partial&space;x}f(x,u)&space;=&space;\left[&space;\begin{array}{cc}&space;1&space;&&space;T\\&space;0&space;&&space;\sin{(x^{2})}&space;\end{array}&space;\right]" title="\frac{\partial}{\partial x}f(x,u) = \left[ \begin{array}{cc} 1 & T\\ 0 & \sin{(x^{2})} \end{array} \right]" /></a>
* <a href="https://www.codecogs.com/eqnedit.php?latex=g(x)&space;=&space;x^{1}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?g(x)&space;=&space;x^{1}" title="g(x) = x^{1}" /></a>
* <a href="https://www.codecogs.com/eqnedit.php?latex=\frac{\partial}{\partial&space;x}g(x)&space;=&space;\left[&space;\begin{array}{cc}&space;1&space;&&space;0&space;\end{array}&space;\right]" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\frac{\partial}{\partial&space;x}g(x)&space;=&space;\left[&space;\begin{array}{cc}&space;1&space;&&space;0&space;\end{array}&space;\right]" title="\frac{\partial}{\partial x}g(x) = \left[ \begin{array}{cc} 1 & 0 \end{array} \right]" /></a>

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

### Full code of example
Example code of tracking with CV model and measurment in polar coordinates:

<a href="https://www.codecogs.com/eqnedit.php?latex=\begin{array}{c}&space;\begin{bmatrix}&space;x_{k&plus;1}\\&space;v^{x}_{k&plus;1}\\&space;y_{k&plus;1}\\&space;v^{y}_{k&plus;1}&space;\end{bmatrix}&space;=&space;\begin{bmatrix}&space;1&space;&&space;T&space;&&space;0&space;&&space;0\\&space;0&space;&&space;1&space;&&space;0&space;&&space;0\\&space;0&space;&&space;0&space;&&space;1&space;&&space;T\\&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;\end{bmatrix}&space;\begin{bmatrix}&space;x_{k}\\&space;v^{x}_{k}\\&space;y_{k}\\&space;v^{y}_{k}&space;\end{bmatrix}&space;&plus;&space;\begin{bmatrix}&space;\nu_{x}\\&space;\nu_{v^{x}}\\&space;\nu_{y}\\&space;\nu_{v^{y}}&space;\end{bmatrix}&space;\\&space;\\&space;\begin{bmatrix}&space;r\\&space;\alpha&space;\end{bmatrix}&space;=&space;\begin{bmatrix}&space;\sqrt{x^{2}&space;&plus;&space;y^{2}}\\&space;\arctan{(\frac{y}{x})}&space;\end{bmatrix}&space;&plus;&space;\begin{bmatrix}&space;\gamma_{r}\\&space;\gamma_{\alpha}&space;\end{bmatrix}&space;\end{array}\" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\begin{array}{c}&space;\begin{bmatrix}&space;x_{k&plus;1}\\&space;v^{x}_{k&plus;1}\\&space;y_{k&plus;1}\\&space;v^{y}_{k&plus;1}&space;\end{bmatrix}&space;=&space;\begin{bmatrix}&space;1&space;&&space;T&space;&&space;0&space;&&space;0\\&space;0&space;&&space;1&space;&&space;0&space;&&space;0\\&space;0&space;&&space;0&space;&&space;1&space;&&space;T\\&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;\end{bmatrix}&space;\begin{bmatrix}&space;x_{k}\\&space;v^{x}_{k}\\&space;y_{k}\\&space;v^{y}_{k}&space;\end{bmatrix}&space;&plus;&space;\begin{bmatrix}&space;\nu_{x}\\&space;\nu_{v^{x}}\\&space;\nu_{y}\\&space;\nu_{v^{y}}&space;\end{bmatrix}&space;\\&space;\\&space;\begin{bmatrix}&space;r\\&space;\alpha&space;\end{bmatrix}&space;=&space;\begin{bmatrix}&space;\sqrt{x^{2}&space;&plus;&space;y^{2}}\\&space;\arctan{(\frac{y}{x})}&space;\end{bmatrix}&space;&plus;&space;\begin{bmatrix}&space;\gamma_{r}\\&space;\gamma_{\alpha}&space;\end{bmatrix}&space;\end{array}\" title="\begin{array}{c} \begin{bmatrix} x_{k+1}\\ v^{x}_{k+1}\\ y_{k+1}\\ v^{y}_{k+1} \end{bmatrix} = \begin{bmatrix} 1 & T & 0 & 0\\ 0 & 1 & 0 & 0\\ 0 & 0 & 1 & T\\ 0 & 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} x_{k}\\ v^{x}_{k}\\ y_{k}\\ v^{y}_{k} \end{bmatrix} + \begin{bmatrix} \nu_{x}\\ \nu_{v^{x}}\\ \nu_{y}\\ \nu_{v^{y}} \end{bmatrix} \\ \\ \begin{bmatrix} r\\ \alpha \end{bmatrix} = \begin{bmatrix} \sqrt{x^{2} + y^{2}}\\ \arctan{(\frac{y}{x})} \end{bmatrix} + \begin{bmatrix} \gamma_{r}\\ \gamma_{\alpha} \end{bmatrix} \end{array}\" /></a>

```cpp
#include <iostream>
#include <cmath>
#include <random>

#include "matplotlibcpp.h"

#include "bayesian_filter.hpp"

namespace plt = matplotlibcpp;

int main(void) {
    bf_io::FilterCalibration calibrations;

    /*
     * Create Model:
     * X(k+1) = [x + T * vx; vx; y + T * vy; vy]
     * Y(k) = [sqrt(x^2 + y^2); arctan(y/x)]
     */
    calibrations.transition = [](const Eigen::VectorXf & state, const float time_delta) {
        auto x = state(0);
        auto vx = state(1);
        auto y = state(2);
        auto vy = state(3);

        auto predicted_x = x + time_delta * vx;
        auto predicted_vx = vx;
        auto predicted_y = y + time_delta * vy;
        auto predicted_vy = vy;

        Eigen::VectorXf transformed_state(4u);
        transformed_state(0) = predicted_x;
        transformed_state(1) = predicted_vx;
        transformed_state(2) = predicted_y;
        transformed_state(3) = predicted_vy;

        return transformed_state;
    };

    calibrations.transition_jacobian = [](const Eigen::VectorXf & state, const float time_delta) {
        std::ignore = state;

        Eigen::MatrixXf jacobian = Eigen::MatrixXf::Identity(4u, 4u);
        jacobian(0, 1) = time_delta;
        jacobian(2, 3) = time_delta;

        return jacobian;
    };

    calibrations.observation = [](const Eigen::VectorXf & state) {
        auto x = state(0);
        auto y = state(2);

        auto range = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        auto azimuth = std::atan2(y, x);

        Eigen::VectorXf observation(2);
        observation(0) = range;
        observation(1) = azimuth;

        return observation;
    };

    calibrations.observation_jacobian = [](const Eigen::VectorXf & state) {
        auto x = state(0);
        auto y = state(2);
        
        Eigen::MatrixXf jacobian = Eigen::MatrixXf::Zero(2u, 4u);
        if ((std::abs(x) < 1e-5f) && (std::abs(y) < 1e-5f)) {
            jacobian(0, 0) = 1.0f;
            jacobian(0, 1) = 1.0f;
            jacobian(1, 2) = 1.0f;
            jacobian(1, 3) = 1.0f;
        } else {
            auto range_sqr = std::pow(x, 2) + std::pow(y, 2);
            auto range = std::sqrt(range_sqr);
            jacobian(0, 0) = x / range;
            jacobian(0, 2) = y / range;
            jacobian(1, 0) = -y / range_sqr;
            jacobian(1, 2) = x / range_sqr;
        }

        return jacobian;
    };

    calibrations.state_dimension = 4u;
    calibrations.measurement_dimension = 2u;
    calibrations.proccess_noise_covariance = 0.1f * static_cast<Eigen::MatrixXf>(Eigen::MatrixXf::Identity(4, 4));

    /* Create filters */
    bf::BayesianFilter ekf_filter(bf_io::FilterType::EKF, calibrations);
    bf::BayesianFilter ukf_filter(bf_io::FilterType::UKF, calibrations);

    /* Run filters */
    bf_io::ValueWithTimestampAndCovariance measurement;
    measurement.timestamp = 0.0;
    measurement.state = {10.0f, 10.0f};
    measurement.covariance.diagonal = {0.1f, 0.01f};
    measurement.covariance.lower_triangle = {0.0f};

    constexpr size_t size = 1000u;

    std::vector<double> ref_x(size, 10.0f);
    std::vector<double> ref_y(size, 0.0f);
    std::vector<double> ekf_x(size);
    std::vector<double> ekf_vx(size);
    std::vector<double> ekf_y(size);
    std::vector<double> ekf_vy(size);
    std::vector<double> ukf_x(size);
    std::vector<double> ukf_vx(size);
    std::vector<double> ukf_y(size);
    std::vector<double> ukf_vy(size);
    std::vector<double> timestamps(size);
    std::vector<double> meas_x(size);
    std::vector<double> meas_y(size);

    std::default_random_engine generator;
    std::normal_distribution<float> distribution_range(0.0f, 0.1f);
    std::normal_distribution<float> distribution_azimuth(0.0f, 0.01f);

    for (auto index = 0u; index < size; index++)
    {        
        measurement.timestamp += 0.01f;
        auto x = 10.0f;
        auto y = 10.0f;
        measurement.state.at(0) = std::sqrt(std::pow(x, 2) + std::pow(y, 2)) + distribution_range(generator);
        measurement.state.at(1) = std::atan2(y, x) + distribution_azimuth(generator);

        timestamps.at(index) = measurement.timestamp;
        meas_x.at(index) = measurement.state.at(0) * std::cos(measurement.state.at(1));
        meas_y.at(index) = measurement.state.at(0) * std::sin(measurement.state.at(1));
        
        // EKF
        ekf_filter.RunFilter(measurement);
        auto ekf_result = ekf_filter.GetEstimation();
        ekf_x.at(index) = static_cast<double>(ekf_result.state.at(0));
        ekf_vx.at(index) = static_cast<double>(ekf_result.state.at(1));
        ekf_y.at(index) = static_cast<double>(ekf_result.state.at(2));
        ekf_vy.at(index) = static_cast<double>(ekf_result.state.at(3));

        // UKF
        ukf_filter.RunFilter(measurement);
        auto ukf_result = ukf_filter.GetEstimation();
        ukf_x.at(index) = ukf_result.state.at(0);
        ukf_vx.at(index) = ukf_result.state.at(1);
        ukf_y.at(index) = ukf_result.state.at(2);
        ukf_vy.at(index) = ukf_result.state.at(3);
    }

    /* Plots */
    plt::figure();
    plt::plot(timestamps, ref_x, "m--", {{"label", "Reference"}});
    plt::plot(timestamps, ekf_x, "b", {{"label", "EKF"}});
    plt::plot(timestamps, ukf_x, "r", {{"label", "UKF"}});
    plt::plot(timestamps, meas_x, "k+", {{"label", "Measurement"}});
    plt::title("x");
    plt::grid();
    plt::legend();
    plt::xlabel("Time [s]");
    plt::ylabel("x [m]");
    plt::show();

    plt::figure();
    plt::plot(timestamps, ekf_vx, "b", {{"label", "EKF"}});
    plt::plot(timestamps, ukf_vx, "r", {{"label", "UKF"}});
    plt::title("vx");
    plt::grid();
    plt::legend();
    plt::xlabel("Time [s]");
    plt::ylabel("vx [m/s]");
    plt::show();
    
    plt::figure();
    plt::plot(timestamps, ref_y, "m--", {{"label", "Reference"}});
    plt::plot(timestamps, ekf_y, "b", {{"label", "EKF"}});
    plt::plot(timestamps, ukf_y, "r", {{"label", "UKF"}});
    plt::plot(timestamps, meas_y, "k+", {{"label", "Measurement"}});
    plt::title("y");
    plt::grid();
    plt::legend();
    plt::xlabel("Time [s]");
    plt::ylabel("y [m]");
    plt::show();

    plt::figure();
    plt::plot(timestamps, ekf_vy, "b", {{"label", "EKF"}});
    plt::plot(timestamps, ukf_vy, "r", {{"label", "UKF"}});
    plt::title("vy");
    plt::grid();
    plt::legend();
    plt::xlabel("Time [s]");
    plt::ylabel("vy [m/s]");
    plt::show();

    return EXIT_SUCCESS;
}
```

It produces following results:
![x](https://user-images.githubusercontent.com/44383270/137217945-2b8cd3cf-8a42-434e-b190-1be38a2bc66e.png)
![vx](https://user-images.githubusercontent.com/44383270/137217961-df911b74-5202-4f96-88a3-6c904ea1de53.png)
![y](https://user-images.githubusercontent.com/44383270/137217967-b13f77ff-8128-4d57-b463-d01fc31de0da.png)
![vy](https://user-images.githubusercontent.com/44383270/137217982-43d13ee3-bff9-4151-bff7-961705f54416.png)

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
* Other:
  * Python 3.8 - for plotting

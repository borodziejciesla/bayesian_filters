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

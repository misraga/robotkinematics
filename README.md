# robotkinematics

A lightweight C++20 library for serial manipulator kinematics using Denavit–Hartenberg parameters.

Built on Eigen. Includes forward kinematics, geometric Jacobians, and damped least-squares inverse kinematics.

---

## Features

- Forward kinematics (DH formulation)
- Space Jacobian
- Body Jacobian (via adjoint transform)
- SE(3) logarithm map
- Damped Least Squares (DLS) IK solver

Twist convention: **[v; ω]** (linear velocity first, angular velocity second).

---

## Requirements

- Linux
- CMake ≥ 3.20
- C++20 compiler
- Eigen3

Install dependencies (Ubuntu/Debian):
```bash
sudo apt install build-essential cmake libeigen3-dev
```
## Build
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```
## Run example
```bash
./build/rk_example
```



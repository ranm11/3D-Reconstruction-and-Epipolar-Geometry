# Monocular Visual Odometry using Epipolar Geometry

This script implements monocular visual odometry (VO) for estimating camera motion from a sequence of images using epipolar geometry principles. It processes video frames from the KITTI dataset, extracts features, computes optical flow, estimates the essential matrix, recovers the relative camera pose, and accumulates the trajectory over time.

## Overview

The script performs the following main steps:
1. **Feature Extraction**: Detects good features to track in each frame using Shi-Tomasi corner detection.
2. **Optical Flow**: Computes sparse optical flow between consecutive frames using the Lucas-Kanade method.
3. **Essential Matrix Estimation**: Estimates the essential matrix using either the 5-point algorithm or by computing the fundamental matrix and converting it to the essential matrix.
4. **Pose Recovery**: Recovers the rotation and translation between camera poses from the essential matrix.
5. **Trajectory Accumulation**: Accumulates the relative poses to build the complete camera trajectory.
6. **Visualization**: Displays the current frame with optical flow vectors and updates a 3D plot of the camera trajectory in real-time.

## Mathematical Background

### Epipolar Geometry Fundamentals

Epipolar geometry describes the geometric relationship between two views of a 3D scene. For two cameras with projection matrices P and P', a point X in 3D space projects to points x and x' in the two images. The epipolar constraint states that x' lies on the epipolar line corresponding to x, and vice versa.

The fundamental matrix F encapsulates this relationship:
$$
\mathbf{x}'^T \mathbf{F} \mathbf{x} = 0
$$

Where $\mathbf{x}$ and $\mathbf{x}'$ are homogeneous coordinates of corresponding points.

### Essential Matrix

The essential matrix E relates the rotation R and translation t between two camera coordinate systems:
$$
\mathbf{E} = [\mathbf{t}]_\times \mathbf{R}
$$

Where $[\mathbf{t}]_\times$ is the skew-symmetric matrix of the translation vector $\mathbf{t}$.

For calibrated cameras, the essential matrix can be computed from the fundamental matrix:
$$
\mathbf{E} = \mathbf{K}'^T \mathbf{F} \mathbf{K}
$$

Where $\mathbf{K}$ and $\mathbf{K}'$ are the camera intrinsic matrices (assumed identical in this monocular setup).

### Pose Estimation from Essential Matrix

Given the essential matrix $\mathbf{E}$, the relative pose $(\mathbf{R}, \mathbf{t})$ can be recovered up to a scale ambiguity. The essential matrix has the property that $\mathbf{E} = \mathbf{U} \boldsymbol{\Sigma} \mathbf{V}^T$, where $\boldsymbol{\Sigma} = \begin{pmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 0 \end{pmatrix}$.

The possible rotations and translations are:
- $\mathbf{R}_1 = \mathbf{U} \mathbf{W} \mathbf{V}^T$, $\mathbf{R}_2 = \mathbf{U} \mathbf{W}^T \mathbf{V}^T$ (where $\mathbf{W}$ is a rotation matrix)
- $\mathbf{t}_1 = \mathbf{U}[:, 2]$, $\mathbf{t}_2 = -\mathbf{U}[:, 2]$

The correct solution is chosen by checking which one results in positive depth for triangulated points.

### Visual Odometry Pipeline

1. **Feature Tracking**:
   - **Corner Detection (Shi-Tomasi)**: Identifies interest points by computing the minimum eigenvalue of the second-moment matrix (structure tensor) over a window:
     $$
     \mathbf{M} = \sum \begin{pmatrix} I_x^2 & I_x I_y \\ I_x I_y & I_y^2 \end{pmatrix}
     $$
     where $I_x$ and $I_y$ are image gradients. Points with eigenvalues above a threshold are selected as corners.
   - **Optical Flow (Lucas-Kanade)**: Tracks feature points between frames by solving for displacement $(\delta x, \delta y)$ that minimizes the sum of squared differences:
     $$
     \min_{\delta x, \delta y} \sum (I(x + \delta x, y + \delta y) - I(x, y))^2
     $$
     Using first-order Taylor approximation: $I(x + \delta x, y + \delta y) \approx I(x,y) + I_x \delta x + I_y \delta y$, leading to the linear system $\mathbf{A}^T \mathbf{A} \mathbf{v} = \mathbf{A}^T \mathbf{b}$ where $\mathbf{v} = (\delta x, \delta y)^T$.

   The resulting optical flow vectors $(\delta x, \delta y)$ establish point correspondences between consecutive frames. In homogeneous coordinates, the corresponding point in the next frame is $\mathbf{x}' = \begin{pmatrix} x + \delta x \\ y + \delta y \\ 1 \end{pmatrix}$. These correspondences satisfy the epipolar constraint and are used in the subsequent essential matrix estimation.

2. **Fundamental Matrix Estimation**:
   - **Epipolar Constraint**: Use the optical flow correspondences $\mathbf{x} = (x, y, 1)^T$ and $\mathbf{x}' = (x + \delta x, y + \delta y, 1)^T$ to estimate the fundamental matrix $\mathbf{F}$ from:
     $$
     \mathbf{x}'^T \mathbf{F} \mathbf{x} = 0.
     $$
   - **Linear System**: Each correspondence gives a linear equation in the nine elements of $\mathbf{F}$:
     $$
     x' x F_{11} + x' y F_{12} + x' F_{13} + y' x F_{21} + y' y F_{22} + y' F_{23} + x F_{31} + y F_{32} + F_{33} = 0.
     $$
     This is assembled into a matrix equation $\mathbf{A} \mathbf{f} = 0$, where $\mathbf{f} = \vec{\mathbf{F}}$ and each row of $\mathbf{A}$ contains the products of coordinates $x'_i x_j$.
   - **Solution and Rank Constraint**: Solve the homogeneous system by finding the right singular vector of $\mathbf{A}$ corresponding to the smallest singular value. Then enforce the rank-2 constraint by performing SVD on the estimated $\mathbf{F}$:
     $$
     \mathbf{F} = \mathbf{U} \mathbf{S} \mathbf{V}^T,
     $$
     and replacing $\mathbf{S}$ with
     $$
     \mathbf{S}' = \begin{pmatrix} s_1 & 0 & 0 \\ 0 & s_2 & 0 \\ 0 & 0 & 0 \end{pmatrix},
     $$
     where $s_1$ and $s_2$ are the two largest singular values.
   - **RANSAC Robust Estimation**: Use RANSAC to fit $\mathbf{F}$ by sampling minimal sets of 8 correspondences, estimating $\mathbf{F}$, and counting inliers that satisfy $|\mathbf{x}'^T \mathbf{F} \mathbf{x}| < \epsilon$. The best model maximizes inlier count.

3. **Essential Matrix Estimation**:
   - **Intrinsic Calibration**: Convert the estimated fundamental matrix to the essential matrix using the camera intrinsic matrix $\mathbf{K}$:
     $$
     \mathbf{E} = \mathbf{K}^T \mathbf{F} \mathbf{K}.
     $$
   - **5-Point Algorithm**: Alternatively, when camera intrinsics are known, estimate $\mathbf{E}$ directly from 5 correspondences using a specialized solver. In either case, $\mathbf{E}$ should satisfy the epipolar constraint for calibrated points.
   - **Inlier Selection**: Use the same RANSAC mask to keep only correspondences consistent with the best essential matrix estimate.

4. **Pose Recovery**:
   - **SVD Decomposition**: Factor $\mathbf{E} = \mathbf{U} \boldsymbol{\Sigma} \mathbf{V}^T$ with
     $\boldsymbol{\Sigma} = \begin{pmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 0 \end{pmatrix}$.
   - **Rotation Candidates**: $\mathbf{R}_1 = \mathbf{U} \begin{pmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 1 \end{pmatrix} \mathbf{V}^T$, $\mathbf{R}_2 = \mathbf{U} \begin{pmatrix} 0 & 1 & 0 \\ -1 & 0 & 0 \\ 0 & 0 & 1 \end{pmatrix} \mathbf{V}^T$
   - **Translation Candidates**: $\mathbf{t}_1 = \mathbf{U}[:, 2]$, $\mathbf{t}_2 = -\mathbf{U}[:, 2]$
   - **Validation via Triangulation**: For each $(\mathbf{R}, \mathbf{t})$ candidate, triangulate 3D points:
     $$
     \mathbf{X} = \arg\min ||\mathbf{x} - \pi(\mathbf{P} \mathbf{X})||^2 + ||\mathbf{x}' - \pi(\mathbf{P}' \mathbf{X})||^2
     $$
     where $\mathbf{P} = \mathbf{K} [\mathbf{I} | \mathbf{0}]$, $\mathbf{P}' = \mathbf{K} [\mathbf{R} | \mathbf{t}]$. Select the pose where most points have positive depth in both cameras.

4. **Pose Integration**:
   - **Relative Transformation**: The transformation from current to next frame is $\mathbf{T}_{rel} = \begin{pmatrix} \mathbf{R} & \mathbf{t} \\ \mathbf{0}^T & 1 \end{pmatrix}$
   - **Global Pose Update**: Compose transformations: $\mathbf{T}_{global}^{k+1} = \mathbf{T}_{global}^k \cdot \mathbf{T}_{rel}^{k \to k+1}$
   - **Trajectory**: Extract camera positions from translation components of $\mathbf{T}_{global}$

### Camera Calibration

The script uses pre-calibrated camera intrinsics:
- Focal length $f = 707.0912$
- Principal point $(c_x, c_y) = (601.8873, 183.1104)$
- Intrinsic matrix $\mathbf{K} = \begin{bmatrix} f & 0 & c_x \\ 0 & f & c_y \\ 0 & 0 & 1 \end{bmatrix}$

### RANSAC and Outlier Rejection

RANSAC (RANdom SAmple Consensus) is used to robustly estimate the essential/fundamental matrix in the presence of outliers. The algorithm:
1. Randomly sample minimal sets of correspondences.
2. Fit the model (E or F).
3. Count inliers (points satisfying the epipolar constraint within a threshold).
4. Repeat and select the model with the most inliers.

The script requires a minimum number of inliers (100) and inlier ratio (20%) for pose updates to be considered reliable.

## Dependencies

- numpy
- matplotlib
- opencv (cv2)

## Usage

Run the script directly:
```bash
python vo_epipolar.py
```

The script will process the KITTI video sequence and display the results in real-time. Press ESC to exit, SPACE to pause.

## Output

- Real-time visualization of optical flow and camera trajectory
- Saved trajectory file: `vo_epipolar.xyz` containing camera positions over time
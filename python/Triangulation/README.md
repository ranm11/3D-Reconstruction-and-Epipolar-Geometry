# Triangulation Implementation

This directory contains `triangulation_implement.py`, which reconstructs 3D points from two calibrated image views using epipolar geometry and linear triangulation.

## What the script does

- Reads corresponding image points from two views.
- Estimates the fundamental matrix `F` using the 8-point algorithm.
- Converts `F` to the essential matrix `E` using the intrinsic matrix `K`.
- Recovers the relative pose between views: rotation `R` and translation `t`.
- Triangulates each matched point pair to compute the homogeneous 3D point `X`.
- Saves the reconstructed 3D points to a `.xyz` file.

## Mathematical details

### 1. Point correspondences and homogeneous coordinates

For a point in image 0 with pixel coordinates $p = (u, v)$ and its matched point in image 1 with coordinates $q = (u', v')$, we represent them in homogeneous coordinates as:

$$
\mathbf{x} = \begin{pmatrix} u \\ v \\ 1 \end{pmatrix}, \qquad
\mathbf{x}' = \begin{pmatrix} u' \\ v' \\ 1 \end{pmatrix}
$$

These correspondences are the input for epipolar geometry estimation.

### 2. Fundamental matrix estimation

The fundamental matrix $\mathbf{F}$ encodes the epipolar geometry between the two views. For corresponding points $\mathbf{x}$ and $\mathbf{x}'$, the epipolar constraint is:

$$
\mathbf{x}'^T \mathbf{F} \mathbf{x} = 0
$$

Expanding this equation gives one linear constraint on the 9 entries of $\mathbf{F}$:

$$
 u' u F_{11} + u' v F_{12} + u' F_{13} + v' u F_{21} + v' v F_{22} + v' F_{23} + u F_{31} + v F_{32} + F_{33} = 0.
$$

With many correspondences, we stack these equations into a linear system:

$$
\mathbf{A} \mathbf{f} = 0,
$$

where $\mathbf{f} = \text{vec}(\mathbf{F})$ is the vectorized form of the fundamental matrix, and each row of $\mathbf{A}$ contains products of the image coordinates.

The solution is found as the singular vector corresponding to the smallest singular value of $\mathbf{A}$.

#### Rank-2 constraint

The fundamental matrix must have rank 2. After solving for $\mathbf{F}$, we enforce this by performing an SVD decomposition:

$$
\mathbf{F} = \mathbf{U} \mathbf{S} \mathbf{V}^T
$$

Then replace the singular values with $\mathbf{S}' = \mathrm{diag}(s_1, s_2, 0)$ and reconstruct:

$$
\mathbf{F} = \mathbf{U} \mathbf{S}' \mathbf{V}^T.
$$

### 3. Essential matrix and camera calibration

For calibrated cameras, the essential matrix $\mathbf{E}$ relates the two camera coordinate systems and removes the effect of intrinsics:

$$
\mathbf{E} = \mathbf{K}^T \mathbf{F} \mathbf{K}
$$

where $\mathbf{K}$ is the intrinsic calibration matrix:

$$
\mathbf{K} = \begin{pmatrix}
 f & 0 & c_x \\
 0 & f & c_y \\
 0 & 0 & 1
\end{pmatrix}.
$$

### 4. Pose recovery from the essential matrix

The essential matrix can be decomposed by SVD as:

$$
\mathbf{E} = \mathbf{U} \boldsymbol{\Sigma} \mathbf{V}^T,
$$

with $\boldsymbol{\Sigma} = \mathrm{diag}(1, 1, 0)$ for a valid essential matrix.

From this decomposition, the relative rotation and translation are recovered as:

- $\mathbf{R} = \mathbf{U} \mathbf{W} \mathbf{V}^T$ or $\mathbf{R} = \mathbf{U} \mathbf{W}^T \mathbf{V}^T$
- $\mathbf{t} \propto \pm \mathbf{U}[:, 2]$

where

$$
\mathbf{W} = \begin{pmatrix}
 0 & -1 & 0 \\
 1 & 0 & 0 \\
 0 & 0 & 1
\end{pmatrix}.
$$

The correct sign and rotation are chosen by checking the reconstructed points for positive depth.

### 5. Linear triangulation

Given the projection matrices for the two views:

$$
\mathbf{P}_0 = \mathbf{K} [\mathbf{I} | \mathbf{0}], \qquad
\mathbf{P}_1 = \mathbf{K} [\mathbf{R} | \mathbf{t}],
$$

and matched image points $\mathbf{p} = (u, v)$ and $\mathbf{q} = (u', v')$, the homogeneous 3D point $\mathbf{X} = (X, Y, Z, W)^T$ is computed by solving:

$$
(u \mathbf{P}_{0,3} - \mathbf{P}_{0,1}) \mathbf{X} = 0, \\
(v \mathbf{P}_{0,3} - \mathbf{P}_{0,2}) \mathbf{X} = 0, \\
(u' \mathbf{P}_{1,3} - \mathbf{P}_{1,1}) \mathbf{X} = 0, \\
(v' \mathbf{P}_{1,3} - \mathbf{P}_{1,2}) \mathbf{X} = 0.
$$

This 4x4 homogeneous system is solved by SVD. The solution is the singular vector corresponding to the smallest singular value.

Convert from homogeneous coordinates to Euclidean coordinates by dividing by $W$:

$$
(X, Y, Z) = \left(\frac{X}{W}, \frac{Y}{W}, \frac{Z}{W}\right).
$$

### 6. Output

The script writes the reconstructed 3D point cloud to `triangulation_implement.xyz`.

## Usage

Run the script from the `python/Triangulation` folder:

```bash
python triangulation_implement.py
```

This README is placed alongside `triangulation_implement.py` as requested, with detailed mathematical descriptions of each major step.
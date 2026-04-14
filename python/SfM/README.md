# Incremental SfM (`sfm_inc.py`) — Detailed Math Pipeline

This document explains the algorithmic pipeline in [sfm_inc.py](sfm_inc.py), with the key equations behind each stage.

## Big Picture

```text
Images -> Keypoints/Descriptors -> Pairwise Matches -> RANSAC(F)
                                             |
                                             v
                                 Best image pair selection
                                             |
                                             v
                                 Essential matrix E + recoverPose
                                             |
                                             v
                                   Initial triangulation (X)
                                             |
                                             v
                              Geometric filtering (depth/parallax)
                                             |
                                             v
                        (Planned) PnP + Triangulation + Bundle Adjustment
```

---

## 1) Camera Parameterization Used in the Script

Each camera is stored as a 9-vector:

$$
\mathbf{c} = [f,\; c_x,\; c_y,\; r_x,\; r_y,\; r_z,\; t_x,\; t_y,\; t_z].
$$

- Intrinsics from `get_camera_mat`:

$$
\mathbf{K} =
\begin{bmatrix}
 f & 0 & c_x \\
 0 & f & c_y \\
 0 & 0 & 1
\end{bmatrix}.
$$

- Rotation from Rodrigues vector $\mathbf{r}=[r_x,r_y,r_z]^\top$:

$$
\mathbf{R} = \text{Rodrigues}(\mathbf{r}).
$$

- Projection matrix from `get_projection_mat`:

$$
\mathbf{P} = \mathbf{K}[\mathbf{R}\;|\;\mathbf{t}],
\qquad \mathbf{t}=[t_x,t_y,t_z]^\top.
$$

A 3D point $\mathbf{X}=[X,Y,Z,1]^\top$ projects to image point $\tilde{\mathbf{x}}$ by

$$
\tilde{\mathbf{x}} \sim \mathbf{P}\mathbf{X}.
$$

---

## 2) Feature Extraction

For each image, BRISK extracts keypoints and binary descriptors:

$$
\mathcal{K}_i = \{\mathbf{x}_{i,k}\}, \qquad
\mathcal{D}_i = \{\mathbf{d}_{i,k}\}.
$$

Matching uses brute-force Hamming distance between binary descriptors.

---

## 3) Pairwise Robust Matching via Fundamental Matrix

Given tentative matches $(\mathbf{x}_{i,k},\mathbf{x}_{j,k})$, the script estimates:

$$
\mathbf{F}_{ij} = \text{findFundamentalMat}(\cdot, \text{RANSAC}).
$$

### Epipolar constraint

For inlier correspondences in homogeneous coordinates:

$$
\mathbf{x}_{j}^\top \mathbf{F}_{ij} \mathbf{x}_{i} = 0.
$$

### Linear DLT form and SVD recovery of $\mathbf{F}$

Yes — after the epipolar constraint, the classical linear solve is:

$$
\mathbf{A}_f\mathbf{f}=0,
$$

where $\mathbf{f}$ is the vectorized form of $\mathbf{F}$:

$$
\mathbf{f}=[F_{11},F_{12},F_{13},F_{21},F_{22},F_{23},F_{31},F_{32},F_{33}]^\top.
$$

For one correspondence $\mathbf{x}_i=[x_i,y_i,1]^\top$, $\mathbf{x}_j=[x_j,y_j,1]^\top$, one row of $\mathbf{A}_f$ is

$$
[x_jx_i,\;x_jy_i,\;x_j,\;y_jx_i,\;y_jy_i,\;y_j,\;x_i,\;y_i,\;1].
$$

Stacking all matches gives $N\times 9$ matrix $\mathbf{A}_f$. Then compute:

$$
\mathbf{A}_f=\mathbf{U}\mathbf{\Sigma}\mathbf{V}^\top,
$$

and choose the right singular vector corresponding to the smallest singular value:

$$
\mathbf{f}=\mathbf{v}_{\min}.
$$

Reshape $\mathbf{f}$ into $3\times3$ to obtain $\mathbf{F}$ (then enforce rank-2, optionally refine).

**Important for this file (`sfm_inc.py`)**: this matrix build and SVD are not written explicitly in Python here, because `cv2.findFundamentalMat(..., cv2.RANSAC)` performs robust estimation internally. So the math is the same, but OpenCV hides the low-level linear algebra.

RANSAC keeps correspondences with small geometric error and rejects outliers.

The script keeps pair $(i,j)$ only if inliers exceed `min_inlier_num`.

---

## 4) Best Initial Pair Selection

Among accepted pairs, one pair is chosen as initializer. Conceptually, this should maximize geometric quality (wide baseline, many valid triangulated points).

In code, after tentative selection, the pair is verified by triangulation quality and bad-point rejection.

---

## 5) Relative Pose from Essential Matrix

For the chosen pair $(i,j)$ with intrinsics $\mathbf{K}$:

$$
\mathbf{E}_{ij} = \text{findEssentialMat}(\mathbf{x}_i,\mathbf{x}_j,\mathbf{K}).
$$

Relation between fundamental and essential matrices:

$$
\mathbf{E} = \mathbf{K}^\top \mathbf{F}\mathbf{K}
$$

(when both cameras share the same $\mathbf{K}$ form used here).

`recoverPose` decomposes $\mathbf{E}$ into relative pose:

$$
(\mathbf{R},\mathbf{t}) = \text{recoverPose}(\mathbf{E},\mathbf{x}_i,\mathbf{x}_j).
$$

Geometrically:

$$
\mathbf{x}_j \sim \mathbf{K}(\mathbf{R}\mathbf{X}+\mathbf{t}),
\qquad
\mathbf{x}_i \sim \mathbf{K}\mathbf{X}.
$$

---

## 6) Initial Triangulation

With two projection matrices

$$
\mathbf{P}_0 = \mathbf{K}[\mathbf{R}_0|\mathbf{t}_0],
\qquad
\mathbf{P}_1 = \mathbf{K}[\mathbf{R}_1|\mathbf{t}_1],
$$

OpenCV triangulates points:

$$
\mathbf{X}_k = \text{triangulatePoints}(\mathbf{P}_0,\mathbf{P}_1,\mathbf{x}_{0,k},\mathbf{x}_{1,k}).
$$

Homogeneous normalization is then applied:

$$
\mathbf{X}_k \leftarrow \frac{1}{w_k}[X_k, Y_k, Z_k, w_k]^\top.
$$

This gives Euclidean 3D coordinates $(X_k,Y_k,Z_k)$.

---

## 7) Geometric Filtering (`isBadPoint`)

The script rejects unstable points using three checks:

1. **Depth bounds**
$$
|Z| > Z_{\text{limit}} \Rightarrow \text{reject.}
$$

2. **Positive depth (chirality)** in both views
$$
z_1 \le 0 \;\text{or}\; z_2 \le 0 \Rightarrow \text{reject.}
$$

3. **Parallax threshold** based on cosine
$$
\cos\theta = \frac{\mathbf{v}_1^\top\mathbf{v}_2}{\|\mathbf{v}_1\|\|\mathbf{v}_2\|}.
$$
If $\cos\theta$ is too large (angle too small), triangulation is ill-conditioned; reject when

$$
\cos\theta > \cos\theta_{\max}.
$$

In code, `max_cos_parallax = cos(10°)`.

---

## 8) What Is Already Implemented vs Planned

### Implemented in `sfm_inc.py`
- Image loading and resizing
- BRISK feature detection/description
- Pairwise descriptor matching
- RANSAC fundamental matrix filtering
- Essential matrix + relative pose recovery
- Two-view triangulation
- Basic geometric validation of reconstructed points

### Planned placeholders in code
- **Next-view selection**
- **PnP for new camera pose**
- **Triangulation of newly observed points**
- **Bundle Adjustment (BA)** over cameras + points

---

## 9) Incremental SfM Mathematical Extension (for the TODO part)

When adding camera $m$ incrementally:

1. Use existing 3D–2D correspondences $(\mathbf{X}_k, \mathbf{x}_{m,k})$.
2. Solve PnP:
$$
(\mathbf{R}_m,\mathbf{t}_m)=\arg\min_{\mathbf{R},\mathbf{t}}\sum_k\left\|\mathbf{x}_{m,k}-\pi\big(\mathbf{K}(\mathbf{R}\mathbf{X}_k+\mathbf{t})\big)\right\|_2^2.
$$
3. Triangulate new tracks seen by camera $m$ and at least one existing camera.
4. Run BA over all active cameras and 3D points:
$$
\min_{\{\mathbf{R}_i,\mathbf{t}_i\},\{\mathbf{X}_k\}}\sum_{(i,k)\in\Omega}\rho\left(\left\|\mathbf{x}_{i,k}-\pi\left(\mathbf{K}_i(\mathbf{R}_i\mathbf{X}_k+\mathbf{t}_i)\right)\right\|_2^2\right),
$$
where $\rho(\cdot)$ is a robust loss (Huber/Cauchy).

---

## 10) Notes About Scale and Gauge

Monocular SfM has inherent scale ambiguity:

$$
(\mathbf{R},\mathbf{t},\mathbf{X})\;\text{and}\;(\mathbf{R}, s\mathbf{t}, s\mathbf{X})
$$

produce identical reprojections for any $s>0$.

So recovered structure is up to a global similarity transform unless external scale constraints are provided.

---

## 11) Mapping to Functions

- Intrinsics matrix: `get_camera_mat`
- Pose vector update: `update_camera_pose`
- Projection matrix: `get_projection_mat`
- Point validity checks: `isBadPoint`
- Pipeline orchestration: `main`

This README reflects the current state of [sfm_inc.py](sfm_inc.py), including both implemented and placeholder stages.

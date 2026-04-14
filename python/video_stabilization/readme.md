# 2D Video Stabilization: Mathematical Pipeline

This document explains, in detail, the stabilization pipeline implemented in [video_stabilization.py](video_stabilization.py).

---

## 0) Big Picture: Data Flow

```
Reference Frame I₀
       ↓
  Detect Corners  (Shi-Tomasi)
       ↓
   ℘₀ = {corners}
       ↓
   For each frame Iₜ:
       ├─→ Optical Flow (Lucas-Kanade)
       │   ↓
       │   ℘ₜ = {tracked points}
       │   ↓
       ├─→ Correspondences: ℘ₜ ↔ ℘₀
       │
       ├─→ RANSAC Robust Estimation
       │   ↓
       │   Hₜ = best homography
       │   (maximizing inlier count)
       │
       ├─→ Perspective Warp
       │   ↓
       │   Î ₜ = WarpPerspective(Iₜ, Hₜ)
       │   (stabilized frame)
       │
       └─→ Display & Repeat
```

This flow repeats for each frame in the video.

---

## 1) Goal

Given a video sequence $\{I_t\}_{t=0}^{T-1}$, estimate a frame-to-reference planar transform and warp each frame so that background motion from camera shake is reduced.

The code uses:
- reference frame: first frame $I_0$
- feature tracking: Lucas-Kanade optical flow
- robust global motion model: homography + RANSAC
- stabilization output: perspective-warped frame

---

## 2) Notation

- $I_0$: reference grayscale frame (first frame)
- $I_t$: current grayscale frame at time $t$
- $\mathbf{p}_i^0 = [x_i^0, y_i^0]^\top$: feature point in reference frame
- $\mathbf{p}_i^t = [x_i^t, y_i^t]^\top$: tracked location in frame $t$
- $\tilde{\mathbf{p}} = [x, y, 1]^\top$: homogeneous image point
- $\mathbf{H}_t \in \mathbb{R}^{3\times 3}$: homography mapping frame $t$ to reference frame

---

## 3) Pipeline Steps

## Step A: Read video and set reference frame

The script loads video frames and picks the first frame as reference:

$$
I_0 \leftarrow \text{first frame of video}
$$

If the frame is RGB/BGR, convert to grayscale:

$$
I_0^{(g)} = 0.299R + 0.587G + 0.114B
$$

(OpenCV performs this conversion internally with `cv.cvtColor(..., cv.COLOR_BGR2GRAY)`.)

---

## Step B: Detect good reference points (Shi-Tomasi)

`cv.goodFeaturesToTrack` returns points with strong local gradient structure.

For a patch around pixel $\mathbf{x}$, define the structure tensor:

$$
\mathbf{M}(\mathbf{x}) = \sum_{\mathbf{u}\in \mathcal{N}(\mathbf{x})}
\begin{bmatrix}
I_x(\mathbf{u})^2 & I_x(\mathbf{u})I_y(\mathbf{u}) \\
I_x(\mathbf{u})I_y(\mathbf{u}) & I_y(\mathbf{u})^2
\end{bmatrix}
$$

Shi-Tomasi score:

$$
R(\mathbf{x}) = \min\big(\lambda_1(\mathbf{M}),\lambda_2(\mathbf{M})\big)
$$

Points with large $R(\mathbf{x})$ are selected as trackable corners:

$$
\mathcal{P}_0 = \{\mathbf{p}_i^0\}_{i=1}^{N}
$$

where in code $N \le 2000$.

---

## Step C: Track points with pyramidal Lucas-Kanade

For each new frame $I_t$, points are tracked from reference frame to current frame using `cv.calcOpticalFlowPyrLK`.

### C.1 Brightness constancy assumption

$$
I_0(\mathbf{x}) \approx I_t(\mathbf{x}+\mathbf{d})
$$

where $\mathbf{d}=[u,v]^\top$ is displacement.

Linearize with first-order Taylor expansion:

$$
I_t(\mathbf{x}+\mathbf{d}) \approx I_t(\mathbf{x}) + \nabla I_t(\mathbf{x})^\top\mathbf{d}
$$

So each pixel in a patch provides:

$$
\nabla I_t(\mathbf{x})^\top\mathbf{d} = I_0(\mathbf{x}) - I_t(\mathbf{x})
$$

Stacking all pixels in a patch gives least-squares normal equations:

$$
\mathbf{A}\mathbf{d} = \mathbf{b}, \quad
\mathbf{A}=\sum \nabla I\nabla I^\top, \quad
\mathbf{b}=\sum \nabla I\,\Delta I
$$

$$
\mathbf{d}=\mathbf{A}^{-1}\mathbf{b}
$$

Pyramids are used to handle larger motions (coarse-to-fine refinement).

Output:

$$
\mathcal{P}_t = \{\mathbf{p}_i^t\}_{i=1}^{N}, \quad \text{status}_i \in \{0,1\}
$$

---

## Step D: Estimate global planar transform (Homography)

Using correspondences $\mathbf{p}_i^t \leftrightarrow \mathbf{p}_i^0$, estimate $\mathbf{H}_t$ with `cv.findHomography(..., cv.RANSAC)`.

Homogeneous relation:

$$
	ilde{\mathbf{p}}_i^0 \sim \mathbf{H}_t\tilde{\mathbf{p}}_i^t
$$

Expanded form (with scale $s_i$):

$$
s_i
\begin{bmatrix}
x_i^0 \\
y_i^0 \\
1
\end{bmatrix}
=
\mathbf{H}_t
\begin{bmatrix}
x_i^t \\
y_i^t \\
1
\end{bmatrix}
$$

### D.1 Why homography?

For distant scenes or approximately planar backgrounds, camera shake induces a projective transform that is well modeled by a homography.

### D.2 Robustness via RANSAC

RANSAC repeatedly:
1. Samples minimal 4 point pairs
2. Computes candidate $\mathbf{H}$
3. Counts inliers using reprojection error threshold

Reprojection error for match $i$:

$$
e_i = \left\|\mathbf{p}_i^0 - \pi\left(\mathbf{H}\tilde{\mathbf{p}}_i^t\right)\right\|_2
$$

where $\pi([x,y,w]^\top) = [x/w, y/w]^\top$.

Inliers satisfy $e_i < \tau$.

Result:

$$
\mathbf{H}_t = \arg\max_{\mathbf{H}}\#\{i: e_i<\tau\}
$$

and an inlier mask is returned.

---

## Step E: Warp frame to stabilized view

The frame is warped by perspective transform:

$$
\hat{I}_t(\mathbf{x}) = I_t\!\left(\pi\left(\mathbf{H}_t^{-1}\tilde{\mathbf{x}}\right)\right)
$$

In practice this is done by `cv.warpPerspective(img, H, (W, H))`, producing a stabilized frame aligned to the reference coordinate system.

---

## Step F: Visualization and diagnostics

The script draws line segments from each tracked point $\mathbf{p}_i^t$ to its reference location $\mathbf{p}_i^0$:

- red: inlier correspondence (supports estimated homography)
- green: outlier correspondence (rejected by RANSAC)

Displayed output:

$$
	ext{display}_t = [I_t \;|\; \hat{I}_t]
$$

concatenating original and stabilized frames side-by-side.

---

## 4) Compact End-to-End Formulation

For each $t\ge 1$:

$$
\mathcal{P}_t = \text{LK}(I_0, I_t, \mathcal{P}_0)
$$

$$
\mathbf{H}_t = \text{RANSAC-Homography}(\mathcal{P}_t \leftrightarrow \mathcal{P}_0)
$$

$$
\hat{I}_t = \text{WarpPerspective}(I_t, \mathbf{H}_t)
$$

This directly matches the implementation sequence in [video_stabilization.py](video_stabilization.py).

---

## 5) Practical Notes and Failure Modes

- If too few reliable tracks exist, homography estimation becomes unstable.
- Strong moving foreground objects can bias $\mathbf{H}_t$ unless RANSAC removes them.
- A single fixed reference frame may cause drift/mismatch over long sequences with large viewpoint change.
- This pipeline estimates camera motion in image space only; it does not recover full 3D motion.

---

## 6) Where Linear DLT and SVD Appear in This Project

This section clarifies an important point:

- The stabilization script [video_stabilization.py](video_stabilization.py) does **not** explicitly build a DLT matrix and call SVD in your code.
- Your reconstruction script [../submission.py](../submission.py) **does** explicitly use linear DLT-style systems and SVD.

## 6.1 Linear DLT for Fundamental Matrix (Eight-Point)

In [../submission.py](../submission.py), function `eightpoint`, each correspondence
$\mathbf{x}_1 = [x_1, y_1, 1]^\top$ and $\mathbf{x}_2 = [x_2, y_2, 1]^\top$
contributes one linear equation from the epipolar constraint:

$$
\mathbf{x}_2^\top \mathbf{F}\,\mathbf{x}_1 = 0.
$$

If
$\mathbf{f} = [F_{11},F_{12},F_{13},F_{21},F_{22},F_{23},F_{31},F_{32},F_{33}]^\top$,
the equation becomes

$$
[x_2x_1,\; x_2y_1,\; x_2,\; y_2x_1,\; y_2y_1,\; y_2,\; x_1,\; y_1,\; 1]\,\mathbf{f} = 0.
$$

Stacking all correspondences yields:

$$
\mathbf{A}_F\mathbf{f}=0.
$$

This is the classical linear DLT-style solve for $\mathbf{F}$:

1. Build $\mathbf{A}_F$ (rows from correspondences)
2. Compute SVD $\mathbf{A}_F = \mathbf{U}\mathbf{\Sigma}\mathbf{V}^\top$
3. Take the right singular vector for smallest singular value: $\mathbf{f}=\mathbf{v}_{\min}$
4. Reshape to $3\times 3$ matrix $\mathbf{F}$

That is exactly what the code does in `eightpoint`.

## 6.2 SVD Basis in Seven-Point

In `sevenpoint`, the same linear system is built from 7 correspondences, and SVD gives a 2D null-space basis:

$$
\mathbf{F}(\alpha)=\alpha\mathbf{F}_1 + (1-\alpha)\mathbf{F}_2.
$$

Then rank-2 is enforced via:

$$
\det(\mathbf{F}(\alpha)) = 0,
$$

which is a cubic equation in $\alpha$.

## 6.3 Linear Triangulation (DLT per 3D Point)

In `triangulate`, each 3D point $\mathbf{X}=[X,Y,Z,1]^\top$ is solved from camera equations:

$$
\mathbf{x}_1 \sim \mathbf{C}_1\mathbf{X},\qquad
\mathbf{x}_2 \sim \mathbf{C}_2\mathbf{X}.
$$

From cross-product constraints, form

$$
\mathbf{A}_X\mathbf{X}=0
$$

with 4 rows (2 from each image). Then SVD is used:

$$
\mathbf{A}_X=\mathbf{U}\mathbf{\Sigma}\mathbf{V}^\top,
\qquad
\mathbf{X}=\mathbf{v}_{\min}.
$$

This is the standard linear triangulation DLT solve.

---

## 7) What Happens in Video Stabilization Instead

In [video_stabilization.py](video_stabilization.py), global motion is estimated as a homography with RANSAC:

$$
	ilde{\mathbf{p}}^0 \sim \mathbf{H}\tilde{\mathbf{p}}^t.
$$

OpenCV (`cv.findHomography`) internally handles the linear estimation and robust fitting. The script itself does not explicitly expose the matrix build + SVD steps.

So, conceptually:

- **Reconstruction file** [../submission.py](../submission.py): explicit linear DLT and explicit SVD calls.
- **Stabilization file** [video_stabilization.py](video_stabilization.py): optimization is mostly hidden inside OpenCV APIs.

---

## 8) Quick Equation Summary

- Epipolar constraint:

$$
\mathbf{x}_2^\top\mathbf{F}\mathbf{x}_1=0
$$

- Essential matrix:

$$
\mathbf{E}=\mathbf{K}_2^\top\mathbf{F}\mathbf{K}_1
$$

- Homography stabilization model:

$$
	ilde{\mathbf{p}}^0 \sim \mathbf{H}\tilde{\mathbf{p}}^t
$$

- SVD null-space solve (generic DLT):

$$
\mathbf{A}\mathbf{z}=0,
\quad
\mathbf{A}=\mathbf{U}\mathbf{\Sigma}\mathbf{V}^\top,
\quad
\mathbf{z}=\mathbf{v}_{\min}
$$


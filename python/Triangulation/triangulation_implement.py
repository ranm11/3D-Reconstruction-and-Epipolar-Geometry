import numpy as np
import cv2 as cv

def triangulatePoints(P0, P1, pts0, pts1):
    Xs = []
    for (p, q) in zip(pts0.T, pts1.T):
        # Solve 'AX = 0'
        A = np.vstack((p[0] * P0[2] - P0[0],
                       p[1] * P0[2] - P0[1],
                       q[0] * P1[2] - P1[0],
                       q[1] * P1[2] - P1[1]))
        _, _, Vt = np.linalg.svd(A, full_matrices=True)
        Xs.append(Vt[-1])
    return np.vstack(Xs).T



def drawMatches(img1, img2, pts0, pts1, title='Matched Points'):
    """Draw matched points between two images"""
    h1, w1 = img1.shape[:2]
    h2, w2 = img2.shape[:2]
    
    # Create a canvas to draw both images side by side
    canvas = np.zeros((max(h1, h2), w1 + w2, 3), dtype=np.uint8)
    canvas[:h1, :w1] = img1
    canvas[:h2, w1:w1+w2] = img2
    
    # Draw circles at matched points and connect them with lines
    for pt0, pt1 in zip(pts0.astype(int), pts1.astype(int)):
        color = tuple(np.random.randint(0, 256, 3).tolist())
        cv.circle(canvas, tuple(pt0), 5, color, -1)
        cv.circle(canvas, (pt1[0] + w1, pt1[1]), 5, color, -1)
        cv.line(canvas, tuple(pt0), (pt1[0] + w1, pt1[1]), color, 1)
    
    cv.imshow(title, canvas)
    cv.waitKey(0)
    return canvas


if __name__ == '__main__':
    f, cx, cy = 1000., 320., 240.
    pts0 = np.loadtxt('data\\image_formation0.xyz')[:,:2]
    pts1 = np.loadtxt('data\\image_formation1.xyz')[:,:2]
    output_file = 'triangulation_implement.xyz'

    # Create simple visualization images (640x480)
    img_width, img_height = 640, 480
    img0 = np.ones((img_height, img_width, 3), dtype=np.uint8) * 200
    img1 = np.ones((img_height, img_width, 3), dtype=np.uint8) * 200

    # Estimate the relative pose of two view
    F, _ = cv.findFundamentalMat(pts0, pts1, cv.FM_8POINT)
    K = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]])
    E = K.T @ F @ K
    _, R, t, _ = cv.recoverPose(E, pts0, pts1)

    # Reconstruct 3D points (triangulation)
    P0 = K @ np.eye(3, 4, dtype=np.float32)
    Rt = np.hstack((R, t))
    P1 = K @ Rt
    X = triangulatePoints(P0, P1, pts0.T, pts1.T)
    X /= X[3]
    X = X.T

    # Visualize matched points
    print("Displaying matched points between two views...")
    drawMatches(img0, img1, pts0, pts1, 'Triangulation: Matched Points')

    # Write the reconstructed 3D points
    np.savetxt(output_file, X)
    print(f"Reconstructed {len(X)} 3D points saved to {output_file}")
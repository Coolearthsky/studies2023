import cv2
import math
import numpy as np

# trying out homography

print("translation")
src = np.array([[0,0],[0,1],[1,1],[1,0]])
dst = np.array([[1,0],[1,1],[2,1],[2,0]])
h, status = cv2.findHomography(src, dst)
print(h)
print(status)

print("rotation around the origin")
src = np.array([[0,0],[0,1],[1,1],[1,0]])
dst = np.array([[0,0],[-1,0],[-1,1],[0,1]])
h, status = cv2.findHomography(src, dst)
print(h)
print(status)

# try homography on the ceiling pics

# these are very high-resolution
ceiling = cv2.resize(cv2.imread("my_ceiling.jpg"), (640,480))
ceiling_rotated = cv2.resize(cv2.imread("my_ceiling_rotated.jpg"), (640,480))

cv2.imshow('ceiling', ceiling)
cv2.imshow('ceiling_rotated', ceiling_rotated)
cv2.waitKey()

ceiling_gray = cv2.cvtColor(ceiling, cv2.COLOR_BGR2GRAY)
ceiling_rotated_gray = cv2.cvtColor(ceiling_rotated, cv2.COLOR_BGR2GRAY)

cv2.imshow('image', ceiling_gray)
cv2.waitKey(0)

# try SIFT

sift = cv2.SIFT_create()
ceiling_keys, ceiling_desc = sift.detectAndCompute(ceiling_gray, None)
ceiling_rotated_keys, ceiling_rotated_desc = sift.detectAndCompute(ceiling_rotated_gray, None)

key_image = cv2.drawKeypoints(ceiling, ceiling_keys, None, color=(0,255,0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
cv2.imshow('sift', key_image)
cv2.waitKey()

#kp_gray, desc_gray = sift.detectAndCompute(ceiling_gray, None)

index_params = dict(algorithm = 0, trees = 5)
search_params = dict()
flann = cv2.FlannBasedMatcher(index_params, search_params)

matches = flann.knnMatch(ceiling_desc, ceiling_rotated_desc, k=2)

good_points = []
for m, n in matches:
    if(m.distance < 0.6*n.distance):
        good_points.append(m)

query_pts = np.float32([ceiling_keys[m.queryIdx]
                 .pt for m in good_points]).reshape(-1, 1, 2)
  
train_pts = np.float32([ceiling_rotated_keys[m.trainIdx]
                 .pt for m in good_points]).reshape(-1, 1, 2)
  
# finding  perspective transformation
# between two planes
matrix, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)


# remember this is a homogeneous matrix
# so R|t
np.set_printoptions(precision=2)
print("HOMOGRAPHY")
print(matrix)

# try the simplest thing imaginable

print("THETA DEGREES")
theta = - math.atan2(matrix[0,1], matrix[0,0]) * 180 / math.pi
print(theta)
translation = [matrix[0,2],matrix[1,2]]
print("TRANSLATION")
print(translation)

K = np.array([[100, 0, 100],[0, 100, 100],[0,0,1]])

# this returns 4 options, we have to choose the right one.
num, Rs, Ts, Ns = cv2.decomposeHomographyMat(matrix, K)

print("num")
print(num)
print("rotation")
print(Rs)
print("translation")
print(Ts)
print("normals")
print(Ns)

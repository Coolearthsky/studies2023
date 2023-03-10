import cv2
import math
import numpy as np

# demonstrate estimateAffine2D

ceiling = cv2.resize(cv2.imread("my_ceiling.jpg"), (640,480))
ceiling_rotated = cv2.resize(cv2.imread("my_ceiling_rotated.jpg"), (640,480))

cv2.imshow('ceiling', ceiling)
cv2.imshow('ceiling_rotated', ceiling_rotated)
cv2.waitKey()

ceiling_gray = cv2.cvtColor(ceiling, cv2.COLOR_BGR2GRAY)
ceiling_rotated_gray = cv2.cvtColor(ceiling_rotated, cv2.COLOR_BGR2GRAY)

sift = cv2.SIFT_create()
ceiling_keys, ceiling_desc = sift.detectAndCompute(ceiling_gray, None)
ceiling_rotated_keys, ceiling_rotated_desc = sift.detectAndCompute(ceiling_rotated_gray, None)

key_image = cv2.drawKeypoints(ceiling, ceiling_keys, None, color=(0,255,0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
key_image_rotated = cv2.drawKeypoints(ceiling_rotated, ceiling_rotated_keys, None, color=(0,255,0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
cv2.imshow('sift', key_image)
cv2.imshow('sift_rotated', key_image_rotated)
cv2.waitKey()

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
  
# actually *partial* affine since we don't need all the degrees of freedom
transmat, _ = cv2.estimateAffinePartial2D(query_pts, train_pts)
print("TRANSMAT")
print(transmat)

print("THETA DEGREES")
theta = - math.atan2(transmat[0,1], transmat[0,0]) * 180 / math.pi
print(theta)
translation = [transmat[0,2],transmat[1,2]]
print("TRANSLATION")
print(translation)

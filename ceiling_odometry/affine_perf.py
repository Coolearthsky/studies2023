import cv2
import timeit
import math
import numpy as np

# measure performance of estimateAffine2D

height = 240 # 480
width = 320 # 640
ceiling = cv2.resize(cv2.imread("my_ceiling.jpg"), (width, height))
ceiling_rotated = cv2.resize(cv2.imread("my_ceiling_rotated.jpg"), (width, height))

ceiling_gray = cv2.cvtColor(ceiling, cv2.COLOR_BGR2GRAY)
ceiling_rotated_gray = cv2.cvtColor(ceiling_rotated, cv2.COLOR_BGR2GRAY)

sift = cv2.SIFT_create()


def doit():

    ceiling_keys, ceiling_desc = sift.detectAndCompute(ceiling_gray, None)
    ceiling_rotated_keys, ceiling_rotated_desc = sift.detectAndCompute(
        ceiling_rotated_gray, None
    )

    index_params = dict(algorithm=0, trees=5)
    search_params = dict()

    flann = cv2.FlannBasedMatcher(index_params, search_params)

    matches = flann.knnMatch(ceiling_desc, ceiling_rotated_desc, k=2)

    good_points = []
    for m, n in matches:
        if m.distance < 0.6 * n.distance:
            good_points.append(m)

    query_pts = np.float32([ceiling_keys[m.queryIdx].pt for m in good_points]).reshape(
        -1, 1, 2
    )

    train_pts = np.float32(
        [ceiling_rotated_keys[m.trainIdx].pt for m in good_points]
    ).reshape(-1, 1, 2)

    transmat, _ = cv2.estimateAffinePartial2D(query_pts, train_pts)
    theta = -math.atan2(transmat[0, 1], transmat[0, 0]) * 180 / math.pi
    translation = [transmat[0, 2], transmat[1, 2]]

    print(f"THETA DEGREES {theta:5.3f} TRANSLATION [{translation[0]:5.3f} {translation[1]:5.3f}]")
    
doit()

n = 100

et = timeit.timeit('doit()', number=n, globals=globals())

print(f"total et {et:5.3f} sec")
print(f"item et {et/n:5.3f} sec")
print(f"rate {n/et:5.3f} fps")

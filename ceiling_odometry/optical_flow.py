import cv2
import numpy as np

# optical flow.  i don't think this is useful.

ceiling = cv2.resize(cv2.imread("my_ceiling.jpg"), (640,480))
ceiling_rotated = cv2.resize(cv2.imread("my_ceiling_rotated.jpg"), (640,480))

cv2.imshow('image', ceiling)
cv2.waitKey(0)

ceiling_gray = cv2.cvtColor(ceiling, cv2.COLOR_BGR2GRAY)
ceiling_rotated_gray = cv2.cvtColor(ceiling_rotated, cv2.COLOR_BGR2GRAY)

cv2.imshow('image', ceiling_gray)
cv2.waitKey(0)

feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )
p0 = cv2.goodFeaturesToTrack(ceiling_gray, mask=None, **feature_params)

lk_params = dict( winSize = (15, 15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                              10, 0.03))

p1, st, err = cv2.calcOpticalFlowPyrLK(
    ceiling_gray, 
    ceiling_rotated_gray,
    p0, None,
    **lk_params)

print(p1)
print(st)
print(err)

mask = np.zeros_like(ceiling)
color = np.random.randint(0, 255, (100, 3))

good_new = p1[st==1]
good_old = p0[st==1]

for i, (new, old) in enumerate(zip(good_new, good_old)):
    a, b = new.ravel()
    c, d = old.ravel()
    a = int(a)
    b = int(b)
    c = int(c)
    d = int(d)
    ceiling = cv2.line(ceiling, (a, b), (c, d), color[i].tolist(), 2)
    ceiling = cv2.circle(ceiling, (a, b), 5, color[i].tolist(), -1)

cv2.imshow('frame', ceiling)
cv2.waitKey(0)

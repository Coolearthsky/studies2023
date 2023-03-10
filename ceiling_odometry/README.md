# Ceiling Odometry

We can get an exact rotation estimate by imaging the ceiling,
sort of like an upside-down mouse.

There are a few ways it could work.

* The simplest (best?) way: use __estimateAffine2D__ using some SIFT points.
* Compute a homography on SIFT points and summarize it.
  Since homography has more degrees of freedom than affine transforms,
  it seems strictly worse.
* Another way, compute the optical flow and then summarize it.
  I think this is way more than we need; it computes a full flow field instead of a rigid transform.

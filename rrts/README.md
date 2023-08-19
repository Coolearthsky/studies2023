# Rapidly-exploring Randomized Trees *

This is adapted from the parallel version in prrts.

I think the parallel stuff makes it too hairy to experiment with,
which defeats the purpose imho.  Our problem-spaces are pretty
simple and our compute power is not too high anyway.

One thread should be fine at a slightly lower frame rate.

<img src="prrts_example.png" width=600 />

This is an example of 4 threads in 20ms, not terrible.
To run it, find the ArenaFrame class and run its main method.
Vscode should put a little "run" button nearby.

<img src="swingup.png" width=600 />

This is an example of a one-joint pendulum swing-up solution,
using LQR cost and gain to calculate feasible links.
To run it, click "simulate robot".

PRRTS adapted from Jeff Ichnowski's project

https://github.com/jeffi/prrts-j

LQR-RRT* adapted from

http://people.csail.mit.edu/tlp/pdf/2012/ICRA12_1657_FI.pdf

https://github.com/MahanFathi/LQR-RRTstar


# TODO:

* attach it to FRC stuff
# Smoothing

How can we smooth out the noise in the velocity observations?

The Linear Filter works fine, though it results in lag of length
exactly equal to the "time constant" which shouldn't be a surprise, and
which probably isn't a big deal.

To reduce the lag, we could run the controller updater more often, as
suggested here:

https://docs.wpilib.org/en/stable/docs/software/convenience-features/scheduling-functions.html

Results of the linear filter example are here:

https://docs.google.com/spreadsheets/d/1nCA9WLhvQL6q_f41DpUDgPfaPy7BuJv7HDFEZsSZhJg/edit#gid=0
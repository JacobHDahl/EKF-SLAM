## EKF-SLAM

Working Extended Kalman Filter (EKF) implementation solving the Simultaneous Localization and Mapping (SLAM) problem. Not finely tuned, but does give good results, however it is not able to run the whole Victoria park dataset in its current state.
## Code:

run_simulated_SLAM.py is a run-file for the simulated dataset.

run_real_SLAM.py is a run-file using the real Victoria park dataset. This dataset i over 60 000 iterations long. With this implementation, running the whole dataset will take hours. Recommended number of iterations is under 20 000. 

## How to run:

Run the chosen run-file in python 3.6 or higher. Recommended to run with -O flag.

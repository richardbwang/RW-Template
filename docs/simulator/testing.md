# Testing on a Computer

`tests/` has tests that run on a computer, not the robot:

- the odometry and distance reset math
- the motion profile, path, pure pursuit and RAMSETE math
- a field simulation where MCL corrects drifting odometry with delayed, noisy sensors and Override field elements in the way
- a check that the simulator's calibration fitter recovers a made-up robot

With a C++ compiler installed (see [Running the Simulator](running.md#one-time-setup-install-a-c-compiler)), run from the project folder:

```bash
sh tests/run-tests.sh
```

GitHub runs these tests, and every auton in the simulator, on each push.

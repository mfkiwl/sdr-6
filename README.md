# SDR (Salih's Dead Reckoning)
## README

Basic three dimensional dead reckoning implementation.

### Running `sdr`

### Building file
[CMake](https://cmake.org/) is a prerequisite to build this program. Simply running `cmake .` && `cmake --build .` from the source directory should do the trick

#### Executing file

`sdr <positive_non_zero> [initial_pose_file=<path_to_yaml>]`

where:
* #1: a mandatory argument containing a path to standard plaintext file where each entry consists of (6 * num_of_sources) + 1 items of data, with the former component consisting of velocities recorded on and along the x y z axis respectively and how long those velocities were recorded for
* #2: a mandatory argument consisting of a positive non-zero integer to inform the program how many sensors are reporting velocity readings - required for sensor fusion
* initial_pose_file: optional argument being path to YAML file (ending in .yml, .yaml) containing an initial position and orientation to start (see data/example_initial_pose.yml)

***

Written in C++, powered by the [Eigen](https://eigen.tuxfamily.org/) library.

See LICENSE for terms of usage.

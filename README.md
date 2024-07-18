# TVMC

The Thrustered Vehicle Motion Controller.

This is a rewrite of the original [TVMC](https://github.com/auvsocietyiiitdm/thrustered_vehicle_motioncontroller.git),    
and is also based on updates from the modified version found [here](https://github.com/Phantom140204/thrustered_vehicle_motioncontroller).

The original TVMC had to included in and directly interfaced with C++ or through python through the provided pybinds, which was tedious, and required a new copy of the source code to be used in every new project.

This is a standalone package that is meant to be used as-is, and can controlled via exposed ros topics.

## Building and Running

This depends on the [`rose_tvmc_msg`](https://github.com/auvsociety/tvmc-msg) package, which contains required message files.    
Messages are provided separately in case you don't need the whole TVMC itself, but still want to interface with it in a separate package.    
Just put that in a directory next to this in the workspace.

You will need to provide [`libInterpolate`](https://github.com/CD3/libInterpolate) to use this package.

To install, clone the repository and build as directed [here](https://github.com/CD3/libInterpolate#cmake).   
`libInterpolate` itself depends on `boost` and `Eigen3`, which you will also need to install.

Sometimes with the default `Eigen3` install, the include paths are not as expected by libInterpolate.   
If this is the case, you should get an error similar to this:
```
fatal error: Eigen/Dense: No such file or directory
```
You can fix this pretty easily by just creating symlinks to get around the problem, as described [here](https://stackoverflow.com/a/25537967).

Basically do:

```bash
# or /usr/local/include, should be in there somewhere
$ cd /usr/include  

# we can see that the library is installed but the path is not as expected
$ ls -d eigen*
eigen3

# symlink your problems away
$ sudo ln -sf eigen3/Eigen Eigen
$ sudo ln -sf eigen3/unsupported unsupported
```

Then you should be good to ```catkin_make``` this.

## Basic Overview

The TVMC will take in either manual thrust vectors for each degree for freedom (surge, sway, heave, pitch, roll and yaw), and will use provided thruster maps to convert these directional/angular movements into thrust values for each thruster.

Additionally, PWM values for these thrust values are computed based on a provided Thrust-to-PWM mapping. This allows for linear control of thrust from the thrusters.

If this is not desired, a simple Thrust-to-PWM map with only two values (MaxPWM-MaxThrust, MinPWM-MinThrust) will cause the PWM Reporter to behave as though there is no Thrust-to-PWM mapping is non-existent.

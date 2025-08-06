# Vex Maps
PROS library enabling precise localization of Robots.

## Goals
* abstract localization class providing localization data
* implementation of linear and arc based odometry
* implementation of Monte Carlo Localization optimized for speed
* optional protobuf-based logging for debugging issues and visualizing routes

## Notes
* The odometry implementation uses cartesian coordinates and standard angles. Because of this some signs in the math are flipped from other implementations such as LemLib's. To keep consistency and familiarity with other libraries the math has been changed such that the tuning guides for other libraries can also be used to find the offsets and orientation of the tracking wheels.

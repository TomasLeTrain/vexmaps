# Documentation
## Particle Filter
## Distance sensor model
### Distance sensor math
To figure out the expected distance the sensor should measure we can use basic trigonometry

where a is the vertical/horizontal distance to the wall, theta is the angle of the robot, and c is the expected measured distance 

The following snippet is what this approach might look like for one of the walls
```cpp
// offsets from the center of the robot rotated by the current angle
Point rotated_offset = ...;
// point from which the distance measures
float sensor_point = rotated_offset + point;

(wall.x - sensor_point.x) / cos(angle)
// or
(wall.y - sensor_point.y) / sin(angle)
```
where wall.x/y are the postive or negative coordinate of each wall.

this however can optimized by precomputing 1/(sin|cos), then multiplying by this new value to avoid a division since multiplication is faster.
this changes the code the look something like
```cpp
(...) * secant
(...) * cosecant
```
To optimize further we can expand sensor_point:
```cpp
(wall.x - sensor_point.x) * secant
->
(wall.x - (rotated_point.x + point.x)) * secant
->
((wall.x - rotated_point.x) - point.x) * secant
```
`(wall.x - rotated_point.x)` does not change based on the particle being evaluated, therefore this can be precomputed as well 

Furthermore for any orientation of the robot it is only possible for a distance sensor to sense up to two distinct walls, one vertical and one horizontal. This not only reduces the number of computations but also eliminates the need for conditional statements when checking for walls. This also allows even more precomputation.
For determining what walls are possible to detect, we can reuse the trigonometric functions that have already been computed
```cpp
horizontal_wall_coord = cosa > 0 ? wall_length : -wall_length;
vertical_wall_coord   = sina > 0 ? wall_length : -wall_length;
```

```cpp
// precomputed values
float sina = sin(angle);
float cosa = cos(angle);
float secant = 1/cosa;
float cosecant = 1/sina;

float horizontal_wall_coord = cosa > 0 ? wall_length : -wall_length;
float vertical_wall_coord   = sina > 0 ? wall_length : -wall_length;

float horizontal_wall_length = horizontal_wall_coord + rotated_point.x;
float vertical_wall_length   = vertical_wall_length  + rotated_point.y;

// compute expected distance for each particle
const Length expected_distance = units::min(
        (horizontal_wall_length - point.x) * this->secant,
        (vertical_wall_length   - point.y) * this->cosecant
        );

```

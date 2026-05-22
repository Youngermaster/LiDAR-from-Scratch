# Glossary

A short, plain-language reference for the terms used across this repo and
its documentation. If a term is missing, please add it; this file is meant
to grow with the project.

### Angle (in a scan)

The bearing of a sample relative to the sensor's body frame, in degrees.
By convention in this repo, 0 degrees points along the +X axis and angles
increase counter-clockwise as seen from above. Different SDKs and ROS
conventions differ; check each experiment header for the convention it
uses.

### Body frame

The coordinate system attached to the sensor itself. The sensor is at the
origin. X and Y axes are fixed relative to the sensor's enclosure. When the
sensor moves through the world, the body frame moves with it.

### DToF (Direct Time of Flight)

The measurement principle the C1 uses. The sensor emits a short laser
pulse, times how long it takes for the reflection to come back, and
multiplies by the speed of light to get a distance. Compare with
**Triangulation** (used by the older A1), which infers distance from where
the reflected spot lands on an internal image sensor.

### Express scan

A packed scan format that fits multiple samples per serial frame. Higher
throughput than the legacy format, at the cost of more complex decoding.
Hidden behind library APIs; see `docs/protocol-notes.md` for an outline.

### Frame (in robotics)

A coordinate system. The "body frame", "world frame", "odom frame", and so
on. Most operations in robotics are transforms from one frame to another.

### LiDAR

Light Detection and Ranging. A sensor that uses laser pulses to measure
distance, typically by ToF.

### Motor PWM

The signal that controls the sensor's rotation speed. On the C1's USB
adapter, this is implemented over the DTR line of the USB-serial bridge,
not as a regular protocol command. The high-level libraries hide that
difference behind a uniform `set_motor_pwm(value)` call.

### Point cloud

A set of points in 2D or 3D space. A single 2D LiDAR scan is a point cloud
of around 360 to 500 points arranged in a near-flat ring. Stacking
consecutive scans over time produces a denser 2D cloud (the same points
seen from many vantage points) or a pseudo-3D cloud (if you treat time or
sensor tilt as the Z axis).

### Polar coordinates

A way to describe a point with an **angle** and a **distance** from a
chosen origin. The native output of any rotating LiDAR is polar.

### Cartesian coordinates

A way to describe a point with **(x, y)** offsets along orthogonal axes
from a chosen origin. Most visualization and downstream processing work
in cartesian coordinates. Converting polar to cartesian is:

```
x = distance * cos(angle_rad)
y = distance * sin(angle_rad)
```

### Quality

A 6-bit per-sample value reported by the C1 indicating the confidence in
that single measurement. Higher is better. Zero quality is meaningful: it
typically means "no return". Treat zero-quality samples as missing data,
not as "obstacle at zero distance".

### Revolution / Scan

One full 360-degree rotation of the sensor, containing every sample
collected during that rotation. The sensor flags the first sample of each
revolution with a `start_flag` bit so the host can group samples correctly.

### RPLIDAR

The product line from SLAMTEC. Models include A1, A2, A3, S1, S2, S3, T1,
C1, and so on. They share a protocol family and a common SDK but differ in
range, sampling rate, and measurement principle.

### Sample rate

The number of distance measurements per second. The C1 advertises 5 kHz.
Sample rate divided by rotation rate gives the angular resolution: 5000
samples per second / 10 revolutions per second = 500 samples per
revolution, roughly one sample every 0.72 degrees.

### Sector

A wedge of the scan, defined by an angular range. "Front sector" might
mean "from -30 degrees to +30 degrees relative to the sensor's forward
direction". Useful for obstacle detection: you do not care about an
obstacle behind you, you care about the one in front.

### Start flag

The bit attached to each sample that marks the beginning of a new
revolution. Group samples between consecutive `start_flag=True` events to
get a full scan.

### ToF (Time of Flight)

The general principle of measuring distance by timing a signal's round
trip. DToF is a specific implementation that times an actual laser pulse.
Indirect ToF (iToF) modulates the laser intensity and measures phase
shift; iToF is common in depth cameras but not in this LiDAR.

### Triangulation LiDAR

The measurement principle used by older RPLIDAR units (A1, A2 partly). The
emitter and the receiver are physically offset; the angle at which the
reflection lands on an internal CMOS image sensor depends on the distance
to the target. Triangulation is cheaper and shorter-range than DToF.

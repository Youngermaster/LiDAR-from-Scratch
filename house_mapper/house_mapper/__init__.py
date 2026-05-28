"""
house_mapper: a small real-time 2D mapping tool for the RPLIDAR C1.

See README.md at the project root for the architecture overview and
usage. The package is organized so each module owns one concern:

  geometry   - Pose2D and 2D rigid transforms
  icp        - point-to-point 2D ICP scan matching
  occupancy  - log-odds occupancy grid with bresenham raytracing
  driver     - low-level RPLIDAR C1 serial protocol
  source     - "scan source" abstractions (live, CSV replay)
  mapper     - the coordinator that ties scan source + ICP + grid
  views      - the three pygame views (map, live, distance)
  app        - the pygame main loop and view switching
"""

__version__ = "0.1.0"

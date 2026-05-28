"""
Shared helpers for the Python experiments.

Per CLAUDE.md, code lives here only when three or more experiments share
it. The motivating case is `rplidar_c1.py`: a small, from-scratch driver
for the SLAMTEC RPLIDAR C1 that replaces the unmaintained `pyrplidar`
package. Every experiment that talks to the sensor uses it.
"""

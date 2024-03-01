
# Explanatory file

This file describes each controller in this package.

> The following controllers are sorted by date of creation (from oldest to newest)

1. Triangulation Controller : uses the Range and Bearing sensor to keep a distance matrix up to date in each robot controller
2. Random Walk Controller : incorporate a simple controller to be replicated in order to change its wheels behaviour
3. Morphogenesis Controller : subscribe to a ROS topic in order to take into consideration high-level information while moving
4. Communication Controller : updated the communication (does not hold a full distance matrix anymore), closer to real operation of modules and allows for simulation of more robots (hold one crashed for two robots)

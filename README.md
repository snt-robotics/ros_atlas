## Overview
The ATLAS node enables cooperative localization of drones in 3D space. Both self-localization and localization of other moving bodies are supported.

The localization works on the basis of fiducial markers. Hence it requires the user to provide sensor readings via a ROS topic of type *MarkerData*.

The ATLAS node performs sensor fusion of these readings in order to optimize the resulting orientation with respect to variance and noise.

## Usage
Some examples are provided with this package (see /config and /launch).

## License
ATLAS is released under the GPLv3.

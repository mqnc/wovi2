# wovi2
Dynamic collision-aware workspace visualization for robotic manipulators

Successor of my [dissertation project](https://github.com/mqnc/wovi). Planned updates:

* permissive license (was bound to GPL by simpleCL before)
* independence from ROS so it can still run without docker 2 years from now
* run on CPU for simplicity
* consider collisions with objects
* support the standard vertically-articulated 6-axis kinematics as well as Universal Robots kinematics
* browser visualization for platform-independence
* depth peeling for correct transparency

# Project Status

First draft; workspace of a UR5e for a single end effector orientation, can be changed via gizmo.

# Usage

Compute server:
```
mkdir build
cd build
cmake ..
make
./main
```

Visualization:
```
python3 -m http.server
```
open `localhost:8000/viewer.html` in a browser

# Credits

[Weighted Companion Cube](https://sketchfab.com/3d-models/portal-weighted-companion-cube-b43c0c192a474374b325a997911754bf)
[Rubik's Cube](https://sketchfab.com/3d-models/the-rubiks-cube-high-poly-by-smakologg-3527bfcda2e1492395142423021c3ff2)
[Basket Ball](https://sketchfab.com/3d-models/basket-ball-6900606bf6be47bbac9cdca845074c66)
[Pallet](https://sketchfab.com/3d-models/pallet-ad8768f522184364af70b56846d10fcf)

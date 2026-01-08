# WoVi2

Dynamic collision-aware workspace visualization for robotic manipulators

Aim of this project is to explore interactive workspace visualization as a tool for robot cell design as well as robot and tool design. It is the successor of my [dissertation project](https://github.com/mqnc/wovi).

# Project Status

The project is currently in a proof of concept state; you can display the workspace of a UR5e robot, drag its end effector and some obstacles around and see how it affects the workspace. Workspace here means the set of all points that the robot tcp can reach in the current end effector orientation and in the current aspect (or "c-sheet" or "uniqueness domain", i.e. branch of ik solutions).

The code was quickly patched together from old fragments mainly as an experiment. If the concept catches on, the whole thing needs a clean rewrite.

# Usage

All dependencies are included, just build and run.

## Setup

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
# from the project root directory, run some http server
python3 -m http.server
```
open `localhost:8000/viewer.html` in a browser

## Controls

\[T\] – translation mode<br>
\[R\] – rotation mode<br>
\[E\] – select the robot end effector<br>
\[1\]-\[5\] – select one of the 5 obstacles<br>
\[Esc\] – deselect everything<br>
\[G\] – move in global coordinates<br>
\[L\] – move in local coordinates<br>
🐭 – move the camera or drag the gizmos around

# License

As the code reuses parts of code from motion planners that were written for a company and only allowed to be published under the PolyForm Noncommercial License, this project must follow suit. If there is enough interest (and funding), these parts can (and should) be rewritten.

# Credits

[Kite Motion Planner](https://github.com/mqnc/kite-motion-planner) (PolyForm NC),
[Gestalt Motion Planner](https://github.com/mqnc/gestalt-motion-planner) (PolyForm NC)

[Bullet Physics](https://github.com/bulletphysics/bullet3) (zlib),
[Marching Cube C++](https://github.com/aparis69/MarchingCubeCpp) (PD/MIT),
[three.js](https://threejs.org/) (MIT),
[depth-peeling-demo](https://github.com/gkjohnson/three-depthpeeling-demo) (MIT),
[CivetWeb](https://github.com/civetweb/civetweb) (MIT),
[tinyxml2](https://github.com/leethomason/tinyxml2) (zlib),
[base64](https://github.com/tobiaslocker/base64) (MIT),
[JSON for Modern C++](https://github.com/nlohmann/json) (MIT),
[stl_reader](https://github.com/sreiter/stl_reader) (BSD-2),
[Universal Robot](https://github.com/ros-industrial/universal_robot/blob/kinetic-devel/ur_kinematics/src/ur_kin.cpp) (BSD-3),
[Universal_Robot_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) (BSD-3)

[Weighted Companion Cube](https://sketchfab.com/3d-models/portal-weighted-companion-cube-b43c0c192a474374b325a997911754bf) (CC BY),
[Rubik's Cube](https://sketchfab.com/3d-models/the-rubiks-cube-high-poly-by-smakologg-3527bfcda2e1492395142423021c3ff2) (CC BY),
[Basket Ball](https://sketchfab.com/3d-models/basket-ball-6900606bf6be47bbac9cdca845074c66) (CC BY),
[Pallet](https://sketchfab.com/3d-models/pallet-ad8768f522184364af70b56846d10fcf) (CC BY),
[Aluminium Profile](https://sketchfab.com/3d-models/aluminium-profile-2020-1f414907f99c4f2f95a5718bd1eeab2c) (CC BY)

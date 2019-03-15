## Cheetah-Software
This repository contains the Robot and Simulation software project.

The common folder contains the common library with dynamics and utilities
The resources folder will contain data files, like CAD of the robot used for the visualization
The robot folder will contain the robot program
The sim folder will contain the simulation program. It is the only program which depends on QT.
The third-party will contain *small* third party libraries that we have modified. This should just be libsoem for Cheetah 3, which Pat modified at one point.

## Build
To build all code:
```
mkdir build
cd build
cmake ..
make -j4
```
This build process builds the common library, robot code, and simulator. If you just change robot code, you can simply run `make -j4` again. If you change LCM types, you'll need to run `cmake ..; make -j4`. This automatically runs `make_types.sh`.

To test the common library, run `common/test-common`. To run the robot code, run `robot/robot`. To run the simulator, run `sim/sim`.

Part of this build process will automatically download the gtest software testing framework and sets it up. After it is done building, it will produce a `libbiomimetics.a` static library and an executable `test-common`.  Run the tests with `common/test-common`. This output should hopefully end with

```
[----------] Global test environment tear-down
[==========] 18 tests from 3 test cases ran. (0 ms total)
[  PASSED  ] 18 tests.
```
## Run simulator
To run the simulator:
1. Open the control board
```
./sim/sim
```
2. In the another command window, run the robot control code
```
./robot/robot ${robot_name} ${target_system}
```
Example)
```
./robot/robot 3 s
```
3: Cheetah 3, m: Mini Cheetah
s: simulation, r: robot


## Dependencies:
- Qt 5.10 - https://www.qt.io/download-qt-installer
- LCM - https://lcm-proj.github.io/
- Eigen - http://eigen.tuxfamily.org
- `mesa-common-dev`
- `freeglut3-dev`

## Install SWIG
```
git clone https://github.com/swig/swig.git

cd swig
./autogen.sh
./configure
make
sudo make install
```
If you have the following error: "./autogen.sh: aclocal: not found", try:
```
sudo apt-get install autotools-dev
sudo apt-get install automake
```

If you have the following error: "make: yacc: Command not found", try:
```
sudo apt-get install bison
```



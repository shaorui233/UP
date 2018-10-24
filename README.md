This repository contains the Robot and Simulation software project.

The common folder contains the common library with dynamics and utilities
The resources folder will contain data files, like CAD of the robot used for the visualization
The robot folder will contain the robot program
The sim folder will contain the simulation program. It is the only program which depends on QT.
The third-party will contain *small* third party libraries that we have modified. This should just be libsoem for Cheetah 3, which Pat modified at one point.

So far, only the common library has been started.  To build it:

```
cd common
mkdir build
cd build
cmake ..
make -j4 # the first compilation will be slow: it also builds the large testing framework
```

Part of this build process will automatically download the gtest software testing framework and sets it up. After it is done building, it will produce a `libbiomimetics.a` static library and an executable `test-common`.  Run the tests with `./test-common`. This output should hopefully end with

```
[----------] Global test environment tear-down
[==========] 18 tests from 3 test cases ran. (0 ms total)
[  PASSED  ] 18 tests.
```



Eventually, the build process will be set up so that building either the simulator or robot will cause the common library to be built and tested automatically.

#!/bin/bash

sudo LD_LIBRARY_PATH=. ldconfig
sudo LD_LIBRARY_PATH=. ldd ./robot
sudo LD_LIBRARY_PATH=. ./robot m r

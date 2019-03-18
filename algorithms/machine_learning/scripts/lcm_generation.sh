#!/bin/bash


GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "${GREEN} Starting Machine Learning LCM type generation...${NC}"

cd ../algorithms/machine_learning/bridge_lcm_types
# Clean
rm */*.hpp

# Make
lcm-gen -x *.lcm
mkdir -p cpp
mv *.hpp cpp

echo -e "${GREEN} Done with Machine Learning LCM type generation${NC}"

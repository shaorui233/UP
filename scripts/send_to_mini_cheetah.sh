#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd ${DIR}/../mc-build/
rm -rf robot-software
mkdir robot-software
cp common/test-common robot-software
cp robot/robot robot-software
find . -name \*.so* -exec cp {} ./robot-software \;
cp ../scripts/run_mc.sh ./robot-software
DATE=$(date +"%Y%m%d%H%M")
scp -r robot-software user@10.0.0.34:~/robot-software-$DATE/


#!/bin/bash
cd ../lcm-types
# Clean
rm *.jar
rm -rf c_lcm_files
rm */*.py
rm */*.pyc
rm */*.java
rm */*.hpp
rm */*.class

# Make
lcm-gen -jcxp *.lcm
cp /usr/local/share/java/lcm.jar .
javac -cp lcm.jar */*.java
jar cf my_types.jar */*.class
mkdir c_lcm_files
mv *.c c_lcm_files
mv *.h c_lcm_files
mkdir java
mv my_types.jar java
mv lcm.jar java

FILES=$(ls */*.class)
echo ${FILES} > file_list.txt




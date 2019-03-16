## Machine Learning Package
This program is designed to make bridge to python code with our simulation and control codes. 

## Run
1. Run simulator
```
./machine_learning/CheetahSim
```
2. Execute python programe. Currently, there is a simple python test code in this folder.
```
python machine_learning/pytest.py
```

## Dependencies
- Swig: http://www.swig.org/download.html
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

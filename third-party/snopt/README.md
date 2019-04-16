To use snopt, the fortran compiler must be installed.
In linux this can be installed via:
````
sudo add-apt-repository ppa:jonathonf/gcc-7.1
sudo apt-get update
sudo apt-get install gfortran-7
````

In mac, you may try the following:
https://www.webmo.net/support/fortran_osx.html

for license, add the following line in your bashrc:
````
export SNOPT_LICENSE=${Cheetah-Software directory}/third-party/snopt/snopt7.lic
````

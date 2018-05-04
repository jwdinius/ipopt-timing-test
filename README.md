# ipopt-timing-test
Timing comparison between IPOPT installations.

## Model Predictive Control and Optimization

## Experiments

For both experiments, I used the synaptic deb for cppad, the auto-differentiator from the coinor project
```
sudo apt install cppad
```

### apt Timing
To get the deb (on Linux): `sudo apt install coinor-libipopt-dev`.  This builds all dependencies, e.g. the MUMPS solver.  To build the project using the library from the deb, you can execute the following (from the project's root):
```
{}:~/work/ipopt_test(master *)$ mkdir build ; cd build ; cmake .. ; make ;
```

This will build the `test_ipopt_install` executable.

```
{}:~/work/ipopt_test/build(master *)$ ./test_ipopt_install 10000

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
Ipopt is released as open source code under the Eclipse Public License (EPL).
For more information visit http://projects.coin-or.org/Ipopt
******************************************************************************

----------------------TIMING STATS-----------------------
runs: 10000, mean: 0.0382954, stddev: 4.95585e-07
```

### MKL Timing

Steps to build IPOPT with Intel MKL:

* get Intel MKL library by signing up on Intel website
* download tgz and extract
* run install.sh script (or the installGUI.sh script).  Install dir is /opt/intel/...
* run `source /opt/intel/parallel_studio_xe_2018.2.046/psxevars.sh intel64` to setup environment
* clone coinor's latest stable git repo (`git clone -b stable/3.12 https://github.com/coin-or/Ipopt.git ${desired_dir_here}`)
* from within `$desired_dir_here`, `mkdir build ; cd build`
* execute this long command 
```
../configure --prefix=/usr/local --disable-pkg-config --with-blas='-Wl,--no-as-needed -L${MKLROOT}/lib/intel64 -lmkl_intel_lp64 -lmkl_core -lmkl_intel_thread -liomp5 -lpthread -lm -ldl' --with-lapack='-Wl,--no-as-needed -L${MKLROOT}/lib/intel64 -lmkl_intel_lp64 -lmkl_core -lmkl_intel_thread -liomp5 -lpthread -lm -ldl' CFLAGS=' -DMKL_LP64 -m64 -I${MKLROOT}/include -O3 -march=native' CXXFLAGS=' -DMKL_LP64 -m64 -I${MKLROOT}/include -O3 -march=native'
```
* run `make`
* run `make test`
* run `make install`

To use this installation, you will need to
* Set `IPOPT_DIR` env variable:  `export IPOPT_DIR=path_to_ipopt_install`
* Ensure that the included `FindIPOPT.cmake` file has lines 41-70 commented out

The two steps outlined above force the linkage of the custom-built IPOPT library using Pardiso and the MKL.

```
{}:~/work/ipopt_test/build(master *)$ ./test_ipopt_install 10000

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
Ipopt is released as open source code under the Eclipse Public License (EPL).
For more information visit http://projects.coin-or.org/Ipopt
******************************************************************************

----------------------TIMING STATS-----------------------
runs: 10000, mean: 0.0524539, stddev: 0.000158158
```

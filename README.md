# IMACS: A performance evaluation framework for IMAge in the Closed-loop Systems

If you are using this simulator in any form in your work, please cite:
```
Sajid Mohamed, Sayandip De, Konstantinos Bimpisidis, Vishak Nathan, Dip Goswami, Henk Corporaal, and Twan Basten, "IMACS: a framework for performance evaluation of image approximation in a closed-loop system," In 8th Mediterranean Conference on Embedded Computing (MECO), 2019.
```
More details regarding the IMACS framework can be found [here](https://www.es.ele.tue.nl/ecs/imacs).

## Installing IMACS
Tested with the following versions:
* OS: Ubuntu 18.04
* g++ (Ubuntu 7.4.0-1ubuntu1~18.04.1) 7.4.0
* llvm: clang+llvm-6.0.0-x86_64-linux-gnu-ubuntu-14.04.tar.xz
* Halide: checkout branch 810a14b1cef3eb99c4051a2a7ca0c50a9872c37c

1. Install dependent libraries
2. Install Halide
3. Install OpenCV
4. Install Eigen
5. Install VREP
6. Install WEBOTS
7. Edit IMACS paths.hpp
 
Initially, clone this repository:
```
git clone https://github.com/sajid-mohamed/imacs.git
cd imacs
pwd
```
For brevity, `$(IMACSROOT)=pwd`, i.e. the path to `imacs` is called as `$(IMACSROOT)`. 

### 1. Dependent libraries
The following libraries might be needed for successful execution.
```
sudo apt-get install libtinfo-dev
sudo apt-get install libjpeg-dev
sudo apt-get install libtiff5-dev
sudo apt-get install libpng-dev
sudo apt-get install jasper
sudo apt-get install libgtk-3-dev
sudo apt-get install libopenblas-dev
sudo apt-get install -y libavcodec-dev
sudo apt-get install -y libavformat-dev
sudo apt-get install libavutil-dev
sudo apt-get install libswscale-dev
sudo apt-get install valgrind
sudo apt-get install cmake
# LibRAW for reversible pipeline
sudo apt-get install -y libraw-dev
# qt4
sudo apt-get install qt4-qmake
sudo apt-get install libqt4-dev
```

### 2. Installing Halide

If already installed, only export the paths of `Halide/bin` and `Halide/distrib/bin` to `LD_LIBRARY_PATH`. 

Note that for our framework, we use a specific checkout version `git checkout 810a14b1cef3eb99c4051a2a7ca0c50a9872c37c`. If you are using the latest version, you may have to adapt the syntax in the source files.

For more details, https://github.com/halide/Halide/blob/master/README.md.

If not already installed, follow the steps below.

Building Halide
===============

1. Install CLANG + LLVM
2. Make in HALIDE_PATH
3. Install Halide

#### 2.1.1 CLANG + LLVM
Have clang+llvm-6.0.0 installed. If already installed, you only need to export the paths CLANG and LLVM_CONFIG.
1. Download the Pre-Built Binary corresponding to your OS from http://releases.llvm.org/download.html.
Recommended version: LLVM 6.0.0
2. Extract the downloaded file into the folder `llvm` in `$(root)/externalApps`. If this folder does not exist `mkdir externalApps`.
```
cd $(IMACSROOT)/externalApps
mkdir llvm
tar -xvf clang+llvm-6.0.0-x86_64-linux-gnu-ubuntu-14.04.tar.xz -C llvm --strip-components=1
```
You need to export the paths (see `FAQ 1` for more details) `CLANG=$(IMACSROOT)/externalApps/llvm/bin/clang` and `LLVM_CONFIG=$(IMACSROOT)/externalApps/llvm/bin/llvm-config`.

#### 2.1.2 Building Halide with make

With `LLVM_CONFIG` set (or `llvm-config` in your path), you should be
able to just run `make` in the root directory of the Halide source tree.
We checkout a specific version as the syntax of codes in `imacs` is based on this version.
```
cd $(IMACSROOT)/externalApps
git clone https://github.com/halide/Halide.git
cd Halide
git checkout 810a14b1cef3eb99c4051a2a7ca0c50a9872c37c
make
```
A successful installation means all the tests are passed.

#### 2.1.3 Installing Halide
Halide does not have a `make install`. To install we use the following command.
```
make distrib
```
This will copy files to the `distrib` folder. 
Export the Halide bin to the library path, i.e. `LD_LIBRARY_PATH=$(ROOT)/externalApps/Halide/distrib/bin:$(ROOT)/externalApps/Halide/bin:$LD_LIBRARY_PATH`.

### 3. Installing OpenCV

If OpenCV is not already installed, follow the OpenCV installation steps from https://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html.

If opencv is already installed, you should be able to locate `opencv*.pc` file. Make sure the `PKG_CONFIG_PATH` points to the file location for `opencv*.pc`.  

### 4. Install eigen
```
cd $(IMACSROOT)/externalApps
git clone https://gitlab.com/libeigen/eigen.git
```
### 5. Install Vrep
Download the linux version from https://coppeliarobotics.com/downloads.
```
cd $(IMACSROOT)/externalApps
mkdir vrep
tar -xvf CoppeliaSim_Edu_V*.tar.xz -C vrep --strip-components=1
```
Generally for local tar file:
```
tar -xvf FILENAME -C FOLDER --strip-components=1
```
### 6. Install Webots
You can install webots using snap.
```
sudo apt install snap
sudo snap install webots
```
Webots is installed in `/snap/webots/current`.

### 7. Edit IMACS paths
Please change paths relative to your own system in the following files
```
$(IMACSROOT)/src/paths.hpp
$(IMACSROOT)/src/Profiling/demosaic-profiling/Makefile
$(IMACSROOT)/src/ReversiblePipeline/Makefile
$(IMACSROOT)/imacs_vrep_framework.pro
$(IMACSROOT)/imacs_webots_framework.pro
```
## Starting IMACS
To start IMACS 
```
bash imacs.sh
```
Follow the instructions.

To speed-up simulation time in webots, you can disable rendering.
In the Webots window, go to the View dropdown menu, and uncheck Rendering (or just use Ctrl+4).

## How to use IMACS? Validating your controller design
1. The controller needs to be designed separately from this toolchain.  
In the current framework, we use a lane keeping assist system (LKAS) example.  
The controller for this is implemented in `src/lateralController/`.  
The current `lateralControl*.*` files implement a lateral controller with sensor-to-actuator delay less than or equal to sampling period. This means that there is one augmented system state.  
If you want to implement a new controller, e.g. pipelined controller, LQG controller, etc., you need to write your own code for `lateralControl*.cpp` and `lateralControl*.hpp`. Further, if delay > period, the `actuate_steering_angle` in `main_IMACS_*.cpp` needs to be updated correctly considering the timing. 
It is recommended to check to the main loop timing in `main_IMACS_*.cpp`. 


## IMACS Documentation
The main page of the doxygen documentation is `~/imacs/doc/mainpage.dox`.
The settings for the doxygen documentation is located at `~/imacs/doc/Doxyfile`.
You can generate doxygen documentation for IMACS code using the following command.
```
make doc
```
This will generate a `doxygen_output` folder inside doc. 
The entire doxygen documentation can be opened in a webpage by opening `~/imacs/doc/doxygen_output/html/index.html`.
To clean the documentation, 
```
make clean_doc
```
## Run IMACS with a different scene
If you want to change the VREP/WEBOTS scene to simulate, follow the instructions in `imacs.sh`.

### Change a VREP scene
1. Copy the scene (e.g. `*.ttt`) to `~/imacs/vrep_scenes` folder.
2. In `imacs.sh`, add an additional option. The following line opens a new vrep scene from:
```
gnome-terminal -- sh -c 'cd externalApps/vrep; ./coppeliaSim.sh ../../vrep_scenes/`*.ttt`; exec bash'
```
### Change a WEBOTS scene
1. Copy the scene (e.g. `*.wbt`) to `~/imacs/webots_scenes` folder.
2. In `imacs.sh`, add an additional option. The following line opens a new webots scene from:
```
gnome-terminal -- sh -c 'cd webots_scenes; webots `*.wbt`; exec bash'
```

## FAQ 1. Exporting paths permanently with sudo rights
```
cd
sudo gedit .bash_profile
```
Add the export path commands in `.bash_profile' as shown below.
```
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/lib/pkgconfig/
```
Save and close `.bash_profile`. Then, open `.bashrc`.
```
sudo gedit .bashrc
```
Add the following line at the end of `.bashrc`.
```
source ~/.bash_profile
```
Save and close the file.
Now, the paths should be exported everytime you open a terminal.

## FAQ 2. Speed-up Webots simulation
Disable 3D Rendering.
* In the Webots window, go to the View dropdown menu, and uncheck Rendering (or just use Ctrl+4).
Increase basicTimeStep.
* Open in a text editor `~/imacs/webots_scenes/*.wbt`. Change "basicTimeStep" to GCF(period, delay) in ms.

# Contact
[Sajid Mohamed](mailto:s.mohamed@tue.nl)



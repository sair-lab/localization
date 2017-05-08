# localization
Localization ROS Package by fusion from multi-sources (UWB, IMU, DVS, SLAM)

Toturial (Only for ROS kinetic on Ubuntu 16.04)

Install dependencies:

    sudo apt-get update

    sudo apt install python-pip

    sudo pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose
    
    sudo easy_install -U statsmodels

    sudo apt install libcholmod3.0.6 libcsparse3.1.4 libsuitesparse-dev python-cvxopt 
    
    sudo apt install ros-kinetic-hector-trajectory-server
    
Install G2O from source. Download g2o to your non-ros source folder, make and install:
  
    git clone https://github.com/RainerKuemmerle/g2o
    cd g2o & mkdir build & cd build
    cmake .. & make -j4
    sudo make install
 
Download uwb_driver to your ros workspace and install its dependencies:

    git clone https://github.com/jeffsanc/uwb_driver.git
    sudo apt-get install ros-kinetic-serial

New support for UWB from bitcrazy:

    please find its driver on:
    https://github.com/wang-chen/lps-ros
    
To use UWB from bitcrazy, comment the following line in the src/localization/localization.h

    #define TIME_DOMAIN

# localization

Localization ROS Package for fusion from multi-sensor (UWB, IMU, Optical Flow, and Visual SLAM)

# Usage
    Only tested with ROS kinetic on Ubuntu 16.04

Install dependencies:

    sudo apt-get update

    sudo apt install python-pip

    sudo pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose
    
    sudo easy_install -U statsmodels

    sudo apt install libcholmod3.0.6 libcsparse3.1.4 libsuitesparse-dev python-cvxopt 
    
    sudo apt install ros-kinetic-hector-trajectory-server
    
Install G2O from source. Download g2o to your non-ros source folder, make and install:
  
    git clone https://github.com/RainerKuemmerle/g2o

Please checkout the following commit for compatibility.

    cd g2o ; git checkout deafc01ee8315b9405351fb145238c5d62f82dc7
    mkdir build ; cd build
    cmake .. ; make -j4
    sudo make install
 
Download uwb_driver to your ros workspace and install its dependencies:

    git clone https://github.com/wang-chen/uwb_driver.git
    sudo apt-get install ros-kinetic-serial

New support for UWB from bitcrazy:

    please find its driver on:
    https://github.com/wang-chen/lps-ros
    
To use UWB from bitcrazy, comment the following line in the CMakeLists.txt

    add_definitions(-DTIME_DOMAIN)
    
# If you are interested in this work, you may cite:

    @inproceedings{wang2018graph,
      title={{Graph Optimization Approach to Localization with Range Measurements}},
      author={Fang, Xu and Wang, Chen and Nguyen, Thien-Minh and Xie, Lihua},
      journal={arXiv preprint arXiv:1802.10276},
      year={2018},
      month=feb
    }
    
    @inproceedings{wang:iros2017,
      author = {Wang, Chen and Zhang, Handuo and  Nguyen, Thien-Minh and Xie, Lihua},
      title = {{Ultra-Wideband Aided Fast Localization and Mapping System}},
      booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
      year = {2017},
      pages={1602--1609},
      month = sep
    }
    
    @inproceedings{wang2018correlation,
      title={{Correlation Flow: Robust Optical Flow using Kernel Cross-Correlators}},
      author={Wang, Chen and Ji, Tete and Nguyen, Thien-Minh and Xie, Lihua},
      booktitle={International Conference on Robotics and Automation (ICRA)},
      year={2018},
      pages = {836--841},
      organization={IEEE},
      month = may
    }

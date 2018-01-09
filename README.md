Real-Time Mapping Based On RGBD Visual Odometry
--
[![GitHub version](https://badge.fury.io/gh/xrandomwalk%2Frgbmapping.png)](http://badge.fury.io/gh/xrandomwalk%2Frgbmapping)

## About

   This project aims to develop a practical system for 3D indoor scene reconstruction, which coherently integrates the 6DoF camera motion estimation and 3D point cloud registration.

   1. About FOVIS

    ```
    In this repo I modfied the original implementation of FOVIS. The new version is in folder rgbdvo.
    Overall the changes include:
    <1> Simplified and reimplemented some parts of the code.
    <2> Only support RGB-D sensor.
    <3> C++11 based.
    <4> SSE computation is default.
    ```

   2. About G2O Framework

   ```shell
   In this repo I modfied a small part of the original G2O. Now it can be compiled with C++11.
   ```

## TODO

  1. Learn about modern 3D Computer Graphics and reimplement the GL render parts by using shader.

## Reference
   * [FOVIS](https://code.google.com/p/fovis/)
   * [g2o](https://https://openslam.org/g2o.html)

## Setup On Mac OSX 10.9

* __Basic__

  ```
  # 1. install command line tools
  xcode-select --install

  # 2. install macports

  # 3. basic dependencies
  sudo port
  install cmake
  install libtool libusb +universal
  install pkgconfig jpeg libpng tiff gtk2 x264 ilmbase openexr tbb
  install qt-mac libQGLViewer qt4-mac
  install py27-matplotlib py27-numpy py27-scipy py27-ipython         # python is optional

  # 4. Install FTDIUSBSerialDriver_v2_2_18

  # 5. Boost 1.55 Library
  cd boost_1_55_0
  ./bootstrap.sh
  ./b2

  # 6. Install the gtest framework
  ```

* __SuiteSparse and METIS__

  These two libraries are used by Eigen and G2O Framework.
  G2O Framework used these two libraries for sparse matrix calculation.

  ```bash
  # SuitSparse
  tar xvf SuitSparse-4.2.1.tar.gz
  cd SuitSparse
  cd SuiteSparse_config
  rm SuiteSparse_config.mk
  mv SuiteSparse_config_Mac.mk SuitSparse_config.mk
  cd ../
  make
  sudo make install

  # METIS
  tar xvf metis-5.1.0.tar.gz
  cd metis-5.1.0
  make config
  sudo make install
  ```

* __Eigen3__

  ```
  cd eigen
  mkdir build
  cd build
  cmake ../
  sudo make install
  ```

* __OpenCV__

  ```bash
  cd opencv-2.4.9
  mkdir build
  cd build
  cmake ../
  ccmake ./
  # WITH_OPENNI ON
  # WITH_TBB ON
  sudo make -j16
  sudo make install
  ```

* __Boost__

  ```bash
  <1> In OSX, use macport.
  <2> In Linux, download src and build.
  ```

## License
Copyright 2014 JZ Xuan <jzxuanuni@gmail.com>. Licensed under the MIT License

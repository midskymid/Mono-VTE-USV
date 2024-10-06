# Mono-VTE-USV
Monocular Vision-based Target Localization and State Estimation Designed for the Jetson Platform
# 1. Introduction
This repository implements the ROS1 version of controller, mainly including the following packages:
* **assets**
* **config_files**
* **trt_weights**
* **utils**

**_Aerial Perspective_**
![mh01](https://github.com/midskymid/Mono-VTE-USV/blob/master/assets/gif/vte_01.gif)

**_Onboard Camera Perspective_**
![mh01](https://github.com/midskymid/Mono-VTE-USV/blob/master/assets/gif/vte_02.gif)
# 2. Prerequisites
* System  
  * Jetpack 5.1.1  
  * ROS1 noetic
* Libraries
  * OpenCV 4.5.4
  * TensorRT 8.5.2.2
  * CUDA 11.4.315
  * cuDNN 8.6.0.166
# 3. Run Mono-VTE-USV
Clone the repository and run:
All configuration files are in the package, **_config_files_**.  
Open four terminals, activate your virtual environment. Take an example  
```
git clone https://github.com/midskymid/Mono-VTE-USV.git
cd Mono-VTE-USV
python vision_enclosing_control.py
```

# 4. Acknowledgements
we referred to parts of the implementations from [LSM_XFeat_trt](https://github.com/midskymid/LSM_XFeat_trt).

# 5. Licence
The source code is released under [GPLv3](https://www.gnu.org/licenses/) license.

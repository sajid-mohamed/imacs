/** @file config_vrep.hpp
 *  @brief The configuration file for IMACS framework using VREP simulator
 */
#ifndef CONFIG_VREP_H_
#define CONFIG_VREP_H_

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>
//!< Header files to include after defining paths
#include "Halide.h"
#include "lateral_Control_VREP.hpp"
#include "lane_detection_vrep.hpp"
#include "image_signal_processing.hpp"
#include "paths.hpp"

using namespace std;

// ------------ defs -------------//
    #define VREP_CAM 1         		//!< select when VREP camera frames are used
    #define SELECT_PERIOD 1		//!< select period (may be redundant due to scenario. [TO DO] check!)
    #define DRAW_SLIDING_WINDOWS 1   	//!< select whether to draw the sliding window while tracking
    #define RE_DRAW_IMAGE 1          	//!< run re-draw image function
//#define DEBUG
//#define DEBUGALL

#endif


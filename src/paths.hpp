/** @file paths.hpp
 *  @brief The header file to store hard-coded paths in files
 */
#ifndef PATHS_H_
#define PATHS_H_

#include <stdio.h>
#include <iostream>
using namespace std;

/// @brief Class for storing the paths
class pathsIMACS {
  public:
    string out_string= "/home/imacs/Desktop/imacs/out_imgs"; //!< Path to store the images;
    string cam_model_path = "/home/imacs/Desktop/imacs/src/ReversiblePipeline/camera_models/NikonD7000/"; //!< Path to the camera model to be used
    string control_csv_dir = "/home/imacs/Desktop/imacs/csv/controllers/"; //!< csv files that contain the controller (automatically generated from MATLAB Code)
    string speedometer_image_path= "/snap/webots/current/usr/share/webots/docs/guide/images/speedometer.png"; //!< speedometer image
    string gold_ref_csv = "/home/imacs/Desktop/imacs/webots_scenes/city_straight_ref.csv"; //!< csv file that contains the reference lane centre for city_straight.wbt
    string city_ref_csv = "/home/imacs/Desktop/imacs/webots_scenes/city_ref.csv"; //!< csv file that contains the reference lane centre for city.wbt
    string lane_out_img_dir = "/home/imacs/Desktop/imacs/out_imgs/"; //!< stores the intermediate images of lane detection
    string csv_dir = "/home/imacs/Desktop/imacs/csv/"; 		//!< stores the csv files
    string results = "/home/imacs/Desktop/imacs/csv/results"; 	//!< stores the result csv files
    pathsIMACS(){} //!< constructor
    ~pathsIMACS(){} //!< destructor
};
#endif

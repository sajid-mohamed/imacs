/** @file webots_api.hpp
 *  @brief The header file for webots API
 */
#ifndef CPP_WEBOTS_API_WEBOTS_API_H_
#define CPP_WEBOTS_API_WEBOTS_API_H_

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include<stdio.h>
#include<string.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<sys/ioctl.h>
#include<unistd.h>
#include<errno.h>
#include <math.h>
#include <fstream>
#include <vector>
#include <utility> // std::pair
#include <sstream>

/// libboost include files
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

/// webots include files
#include <webots/Camera.hpp> 		//#include <webots/camera.h>
#include <webots/Device.hpp> 		//#include <webots/device.h>
#include <webots/Display.hpp> 		//#include <webots/display.h>
#include <webots/GPS.hpp> 		//#include <webots/gps.h>
#include <webots/vehicle/Driver.hpp> 	//#include <webots/vehicle/driver.h>
#include <webots/Robot.hpp> 		//#include <webots/robot.h>

#include "paths.hpp"
#include "config_webots.hpp"

using namespace std;
using namespace webots;
using namespace cv;

/// @brief Class for webots API. Inherits the base class pathsIMACS to load the paths to image.
class webotsAPI : pathsIMACS {
public:
    /// Flags to enable various 'features' 
    bool enable_display = false; //!< flag for whether the display is enabled in webots scene
    bool has_camera = false;	 //!< flag for whether the webots scene has a camera
    bool has_gps = false;	 //!< flag for whether the webots scene has a gps enabled

    /// Variables required for camera object
    Camera *camera_front; 	/// WbDeviceTag camera_front; Device tag for the front camera
    int camera_width = -1; 	/// width of the camera
    int camera_height = -1;	/// height of the camera
    double camera_fov = -1.0;	/// camera field-of-view

    /// Variables for gps --> to get location
    GPS *gps; 					/// WbDeviceTag gps;
    double gps_coords[3] = {0.0, 0.0, 0.0};	/// x,y,z GPS coordinates
    double gps_speed = 0.0;			/// speed calculated from GPS
    std::vector<long double> positionx;		/// position x (x-coordinate in city.wbt)
    std::vector<long double> positiony;		/// position y (z-coordinate in city.wbt)
    enum { X, Y, Z };   //!< to be used as array indices for GPS coordinates

    /// For speedometer in webots
    Display *display; 			/// WbDeviceTag display; speedometer display
    int display_width = 0;		/// speedometer display width
    int display_height = 0;		/// speedometer display height
    ImageRef *speedometer_image = NULL; /// speedometer image

    /// Vehicle steering variables
    double steer90 = 0.0;	 //!< steer value for 90 degree turns
    double offset_value = 100.0; //!< offset value for 90 degree turns.
    double steering_angle = 0.0; //!< vehicle steering angle to actuate

    /// Speed and image quality (if implementing compression)
    double speed = 0.0;		//!< vehicle speed
    int image_quality = 100;	//!< image quality is set as 100 (highest quality possible). 

    /**  @brief C++ implementation for converting webots camera image to Matrix. From RGB to BGR.
         @param[in] camera          The camera device (tag/id)
         @param[in] image           The image from the camera (3 colour - RGB)
	 @return    the matrix format (of the image in BGR)
      */
    cv::Mat webots_img_2_Mat(Camera *camera, const unsigned char *image);
public:
    /// @brief constructor for webotsAPI
    webotsAPI(){
    }

    /// @brief destructor for webotsAPI
    ~webotsAPI(){
    }

    /**  @brief C++ implementation for setting the cruising vehicle speed in webots.
         @param[in] driver          The webots driver (id)
         @param[in] speed           The cruising vehicle speed to be set in webots
    */	
    void set_speed(Driver *driver, long double speed);

    /**  @brief C++ implementation for setting the vehicle steering angle in webots.
         @param[in] driver          The webots driver (id)
         @param[in] wheel_angle     The vehicle steering angle to be set in webots
    */	
    void set_steering_angle(Driver *driver, double wheel_angle);

    /**  @brief C++ implementation to compute GPS speed in webots.
         @param[in] gps          The webots GPS device (tag/id)
    */	
    void compute_gps_speed(GPS *gps);

    /**  @brief C++ implementation to compute current GPS coordinates of the vehicle in webots.
         @param[in] gps          The webots GPS device (tag/id)
    */	
    void compute_gps_coords(GPS *gps);

    /**  @brief C++ implementation for updating the speedometer display in webots.
         @param[in] display         The speedometer display (tag/id) in webots
         @param[in] driver          The webots driver (id)
    */	
    void update_display(Display *display, Driver *driver);

    /**  @brief C++ implementation for vehicle actuation in webots
         @param[in] driver          The webots driver (id)
         @param[in] steering_angle  The computed steering angle by the controller
    */	
    void sim_actuate(Driver *driver, long double steering_angle);

    /**  @brief C++ implementation to read CSV file with 2 columns into a vector of <vector<double>> 
         @param[in] filename       The csv filename to read from
	 @return    2 column csv file contents are translated into a vector of <vector<double>>
      */
    vector<vector<double>> read_csv(string filename);

    /**  @brief C++ implementation to get the deviation yL from the given (x,y) coordinates of the vehicle and pre-computed reference (GPS coordinates of the centre of lane)
         @param[in] ref           Reference (csv) filename which contains 2 column GPS coordinates of the lane centres of the entire webots scene
         @param[in] positionx     x-position vertex of vehicle
         @param[in] positiony     y-position vertex of vehicle
         @return    the lateral deviation (yL) for the given (x,y) positions
         @note the reference file `ref` needs to be computed apriori and is not part of the current toolchain
      */
    double getyL(string ref, double positionx , double positiony);

    /**  @brief C++ implementation to calculate the current actual lateral deviation of the vehicle from the centre of the lane in webots.
         @param[in] gps          The webots GPS device (tag/id)
         @param[in] world_encode The webots wbt file. 1: city_straight.wbt, 2: city.wbt
	 @return the current lateral deviation (yL)
    */	
    long double calculate_actual_deviation(GPS *gps, int world_encode=1);

    /**  @brief C++ implementation to initialise the webots scene.
         @param[in] driver          The webots driver (id)
         @param[in] update_period   The update period/refresh rate for the camera/GPS
    */	
    void initialise(Driver *driver, float update_period);
};

#endif

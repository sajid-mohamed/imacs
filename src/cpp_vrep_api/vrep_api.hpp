/** @file vrep_api.hpp
 *  @brief The header file for VREP Remote API interface when using IMACS framework with VREP simulator
 */
#ifndef CPP_VREP_API_VREP_API_H_
#define CPP_VREP_API_VREP_API_H_

#include <iostream>
#include <cassert>  
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include "lkas_model.hpp"
#include "paths.hpp"
#include "config_vrep.hpp"
#include "utils.hpp"

extern "C" {
    #include "extApi.h"
}
/// @brief Class with vrepAPI utilities
class vrepAPI : lkasModel,utils {
private:
    // private members
    int m_clientID, m_ping_time, m_cam, m_car, m_floor; /// client ID of the API connected to VREP
							/// handles to ping time, camera, car and floor
    int m_resolution[2]; /// resolution of the image
    simxUChar* m_image;  /// handle to capture image from the camera in VREP
    int m_nakedCar_steeringLeft, m_nakedCar_steeringRight; /// handles to steer the car
    int m_nakedCar_motorLeft, m_nakedCar_motorRight; /// handles to move the car
    simxFloat m_position[3]; /// position of the car
    double m_desired_wheel_rot_speed;  /// desired car wheel rotation speed
    // private methods

    /**  @brief C++ implementation for converting VREP camera image to Matrix. From RGB to BGR.
	 @return    the matrix format (of the image in BGR)
      */
    cv::Mat vrep_img_2_Mat();
public:
    /// constructor
    vrepAPI();
    /// destructor
    ~vrepAPI();
    // public methods

    /**  @brief C++ implementation for enforcing synchronous delay in VREP simulation
	 @param[in] time_step 	The number of time steps to delay the simulation. The parameters are same as the previous state.
      */
    void sim_delay(int time_step);

    /**  @brief C++ implementation for capturing image frame from VREP in matrix format
	 @return 	The image captured by the camera in VREP scene in Matrix format
      */
    cv::Mat sim_sense();

    /**  @brief C++ implementation for enforcing the steering angle in the vehicle in VREP simulation
	 @param[in] steering_angle 	The steering angle of the steering wheel to apply
      */
    void sim_actuate(long double steering_angle);
};

#endif

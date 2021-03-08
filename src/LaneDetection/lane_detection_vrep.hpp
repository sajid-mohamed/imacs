/** @file lane_detection_vrep.hpp
 *  @brief The header file for lane detection when using IMACS framework with VREP simulator
 */
#ifndef LANEDETECTION_LANE_DETECTION_H_
#define LANEDETECTION_LANE_DETECTION_H_

#include <iostream>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "Halide.h"
#include "polyfit.hpp"
#include "config_vrep.hpp"
#include "paths.hpp"

/// @brief Class for lane Detection in VREP. Inherits the base class pathsIMACS to load the paths to image.
class laneDetection : pathsIMACS {
private:

    /**  @brief C++ implementation of defining BEV transformation points
           Vertices for the BEV transformation are hard-coded in this function.
         @return    BEV transformation's source and destination vertices
      */
    std::vector<std::vector<cv::Point2f>> get_bev_points();
	
    /**  @brief C++ implementation of Bird's Eye View (BEV) Transformation
           Gets the BEV transformation points, calculates the homography matrix and performs the BEV transformation
         @param[in] 	src            Source image matrix
         @param[in,out] dst            Destination image matrix
      */
    void bev_transform(cv::Mat& src, cv::Mat& dst);
	
    /**  @brief C++ implementation of (Reverse) Bird's Eye View (BEV) Transformation
           Gets the BEV transformation points, calculates the reverse homography matrix and performs the BEV transformation
         @param[in,out] 	src             Source image matrix
         @param[in] dst         Destination 	image matrix
      */
    void bev_rev_transform(cv::Mat& src, cv::Mat& dst);
	
    /**  @brief C++ implementation of Sliding window lane tracking
           Find the lane in the image
         @param[in] 	src            Source image matrix
	 @return lane points in the image
      */
    std::vector<std::vector<cv::Point>> sliding_window_lane_tracking(cv::Mat& src);

    /**  @brief C++ implementation of lateral deviation calculation
           From the given lane points, polyfits the lanes and then finds the deviation of the car from the centre of the lane at the look-ahead distance
         @param[in] 	left_lane_inds            left lane (points)
         @param[in] 	right_lane_inds           right lane (points)
	 @return lateral deviation of the vehicle with the current heading at the look-ahead distance
      */
    std::vector<long double> calculate_lateral_deviation(std::vector<cv::Point> left_lane_inds, 
                                            std::vector<cv::Point> right_lane_inds);

    /**  @brief C++ implementation to overlay the detected lanes onto the original image
           
         @param[in] 	lanes            	detected lanes
         @param[in] 	src           		source image
         @param[in,out]	img_roi           	Image with only the RoI
         @param[in] 	img_warped           	RoI image after BEV transform
         @param[in,out]	img_detected_lanes      Image with the detected lanes
         @param[in,out]	draw_lines           	Image with lanes drawn
         @param[in,out]	diff_src_rebev          Image with diff_src reverse BEV
         @param[in,out]	img_rev_warped          Original image with lines overlayed
      */
    void lane_identification(std::vector<std::vector<cv::Point>> lanes, cv::Mat& src, cv::Mat& img_roi, 
                            cv::Mat& img_warped, cv::Mat& img_detected_lanes, 
                            cv::Mat& draw_lines, cv::Mat& diff_src_rebev, cv::Mat& img_rev_warped);
public:
    /// constructor
    laneDetection();
    /// destructor
    ~laneDetection();

    /**  @brief C++ implementation of lane detection pipeline
           Scales the image, does BEV transformation, Gray-scale conversion, White masking, Image thresholding, Sliding window lane tracking and lateral deviation calculation. Images are also stored in lane_out_img_dir, if RE_DRAW_IMAGE is defined.
         @param[in] src            Source image Matrix
         @return    the calculated lateral deviation yL
      */
    long double lane_detection_pipeline(cv::Mat src);
};

#endif

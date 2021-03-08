/** @file lane_detection_webots.hpp
 *  @brief The header file for lane detection when using IMACS framework with WEBOTS simulator
 */
#ifndef LANEDETECTION_LANE_DETECTION_WEBOTS_H_
#define LANEDETECTION_LANE_DETECTION_WEBOTS_H_

#include <iostream>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "Halide.h"
#include "polyfit.hpp"
#include "config_webots.hpp"

/// @brief Class for lane detection WEBOTS. Inherits the base class pathsIMACS to load the paths to image.
class laneDetectionWEBOTS : pathsIMACS {
private:
    float x1, y1, x2, y2, x3, y3, x4, y4; 		/// RoI selection: (xi,yi) are the four co-ordinates of the RoI
    float x1d, y1d, x2d, y2d, x3d, y3d, x4d, y4d;	/// Bird's eye view (BEV) destination co-ordinates for BEV transformation
	
    /**  @brief C++ implementation of Bird's Eye View (BEV) Transformation
           Gets the BEV transformation points, calculates the homography matrix and performs the BEV transformation
         @param[in] 	src            Source image matrix
         @param[in,out] dst            Destination image matrix
         @param[in] 	world_encode   The Webots scene- 1: city_straight, 2: city_straight_night
      */
    void bev_transform(cv::Mat& src, cv::Mat& dst, int world_encode);
	
    /**  @brief C++ implementation of (Reverse) Bird's Eye View (BEV) Transformation
           Gets the BEV transformation points, calculates the reverse homography matrix and performs the BEV transformation
         @param[in,out]	src            Source image matrix
         @param[in] 	dst            Destination image matrix
         @param[in] 	world_encode   The Webots scene- 1: city_straight, 2: city_straight_night
      */
    void bev_rev_transform(cv::Mat& src, cv::Mat& dst, int world_encode);

    /**  @brief C++ implementation of defining BEV transformation points
           Vertices for the BEV transformation are hard-coded in this function.
         @param[in] world_encode   The Webots scene- 1: city_straight, 2: city_straight_night
         @return    BEV transformation's source and destination vertices
      */
    std::vector<std::vector<cv::Point2f>> get_bev_points(int world_encode);

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
         @param[in] 	ref_tune_hardcoded        hardcoded tuning parameter obtained from trial and error
	 @return lateral deviation of the vehicle with the current heading at the look-ahead distance
      */
    long double calculate_lateral_deviation(std::vector<cv::Point> left_lane_inds,
                                        std::vector<cv::Point> right_lane_inds, float ref_tune_hardcoded);

    /**  @brief C++ implementation to overlay the detected lanes onto the original image
           
         @param[in] 	lanes            	detected lanes
         @param[in] 	src           		source image
         @param[in,out]	img_roi           	Image with only the RoI
         @param[in] 	img_warped           	RoI image after BEV transform
         @param[in,out]	img_detected_lanes      Image with the detected lanes
         @param[in,out]	draw_lines           	Image with lanes drawn
         @param[in,out]	diff_src_rebev          Image with diff_src reverse BEV
         @param[in,out]	img_rev_warped          Original image with lines overlayed
         @param[in] 	world_encode   		The Webots scene- 1: city_straight, 2: city_straight_night
      */
    void lane_identification(std::vector<std::vector<cv::Point>> lanes, cv::Mat& src, cv::Mat& img_roi,
                        cv::Mat& img_warped, cv::Mat& img_detected_lanes,
                        cv::Mat& draw_lines, cv::Mat& diff_src_rebev, cv::Mat& img_rev_warped, int world_encode);
public:
    /// constructor
    laneDetectionWEBOTS();
    /// destructor
    ~laneDetectionWEBOTS();

    /**  @brief C++ implementation of lane detection pipeline
           Scales the image, does BEV transformation, Gray-scale conversion, Image thresholding, Sliding window lane tracking and lateral deviation calculation. Images are also stored in lane_out_img_dir, if RE_DRAW_IMAGE is defined.
         @param[in] src            Source image Matrix
         @param[in] world_encode   The Webots scene- 1: city_straight, 2: city_straight_night
         @return    the calculated lateral deviation yL
      */
    long double lane_detection_pipeline(cv::Mat src, int world_encode);
};

#endif

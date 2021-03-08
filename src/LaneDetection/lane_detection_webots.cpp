/** @file lane_detection_webots.cpp
 *  @brief The source file for lane detection when using IMACS framework with WEBOTS simulator
 
 *  Copyright (c) 2020 sayandipde
 *  Eindhoven University of Technology
 *  Eindhoven, The Netherlands
 *
 *  Name            :   lane_detection.cpp
 *
 *  Authors         :   Sayandip De (sayandip.de@tue.nl)
 *			Sajid Mohamed (s.mohamed@tue.nl)
 *
 *  Date            :   March 26, 2020
 *
 *  Function        :   calculate lateral deviation (yL) from ISP output
 *
 *  History         :
 *      26-03-20    :   Initial version.
 *	28-02-21    :   Adapted with template functions (Sajid)
 *
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "lane_detection_webots.hpp"

using namespace cv;
using namespace std;
using namespace std::chrono; 

/// constructor
laneDetectionWEBOTS::laneDetectionWEBOTS() {}
/// destructor
laneDetectionWEBOTS::~laneDetectionWEBOTS() {}

long double laneDetectionWEBOTS::lane_detection_pipeline(Mat src_in, int world_encode){
	setUseOptimized(true);

	Mat src, img_warped, img_gray, img_lanes;

	/// Scale src
	copyMakeBorder(src_in, src, 256, 0, 0, 0, BORDER_CONSTANT, Scalar(0)); // Top zeros (black portion at the top)

	/// BEV Transform: get_warped_image
	bev_transform(src, img_warped, world_encode);
	imwrite(lane_out_img_dir+"img_warped.png", img_warped);

    	/// Grayscale conversion: rgb2gray
    	cvtColor(img_warped, img_gray, COLOR_RGB2GRAY); // Gray-scale conversion [SD]

    	/// Image Thresholding: get_threshold_img
	int threshold_value = 0;
	int max_value = 255;
	threshold(img_gray, img_lanes, threshold_value, max_value, THRESH_BINARY | THRESH_OTSU);

    	/// Perform sliding window lane tracking
    	vector<vector<Point>> lanes = sliding_window_lane_tracking(img_lanes);
	
#ifdef DEBUGALL
	cout << "\t[DEBUGALL]\t[laneDetectionWEBOTS::lane_detection_pipeline] No. of lane-pixels: " << lanes[0].size() << " and " << lanes[1].size() << endl; 
#endif	
	
	if ((lanes[0].size() <=1) || (lanes[1].size() <=1) ) /// no lanes detected
		throw out_of_range("[IMACS ERROR] No lanes detected in the camera image!\nMaybe the car went out of the lane.\nIf you believe everything else is correct, it could be that the Region-of-Interest need to be modified. You can change it in laneDetectionWEBOTS::get_bev_points().\nTo Debug this, use live_plot.py to see exactly where you are failing.");

	/// Calculate lateral deviation
	long double yL=0;
	if (world_encode == 2)
		yL = calculate_lateral_deviation(lanes[0], lanes[1], WEBOTS_REF_TUNE_NIGHT);
	else
		yL = calculate_lateral_deviation(lanes[0], lanes[1], WEBOTS_REF_TUNE_STRAIGHT);								  

#ifdef RE_DRAW_IMAGE
	/// lane identification -> reverting back to original image
	Mat img_detected_lanes, draw_lines, diff_src_rebev, img_rev_warped, img_roi;
	lane_identification(lanes, src, img_roi, img_warped, img_detected_lanes, draw_lines, diff_src_rebev, img_rev_warped, world_encode);
	Rect out_roi = Rect(0, 256, 512, 256); // cv::Rect rect(x, y, width, height); 
	Mat cropped = Mat(img_detected_lanes, out_roi);
	imwrite(lane_out_img_dir+"img_roi.png", img_roi);
	imwrite(lane_out_img_dir+"img_diff_src.png", diff_src_rebev);
	imwrite(lane_out_img_dir+"img_output.png", cropped); 
	//imwrite(lane_out_img_dir+"img_draw_lines.png", draw_lines); 
	imwrite(lane_out_img_dir+"img_rev_warped.png", img_rev_warped); 
	imwrite(lane_out_img_dir+"img_gray.png", img_gray);
	imwrite(lane_out_img_dir+"img_binary.png", img_lanes);
#endif
	
#ifdef DEBUGALL
	cout << "\t[DEBUG]\t[laneDetectionWEBOTS::lane_detection_pipeline] Lateral_deviation yL= " << yL << " m" << endl;
#endif
		
    	return yL;
}

vector<vector<Point2f>> laneDetectionWEBOTS::get_bev_points(int world_encode){
    	/// declare variables
    	vector<vector<Point2f>> vertices(2);
    	vector<Point2f> src_vertices(4), dst_vertices(4);
	
#ifdef WEBOTS_CAM
	int warp_offset = 70;
	x1 = 60; y1 = 512; x2 = 300; y2 = 512; x3 = 160; y3 = 447; x4 = 270; y4 = 447;
	if (world_encode == 7) { /// city_straight_snow
		x1 = 60; y1 = 512; x2 = 300; y2 = 512; x3 = 160; y3 = 447; x4 = 275; y4 = 447;
	}
	if (world_encode == 4) { /// city_straight_dawn
		x1 = 60; y1 = 512; x2 = 300; y2 = 512; x3 = 160; y3 = 447; x4 = 280; y4 = 447;
	}
	x1d = 50 + warp_offset; y1d = 512; x2d = 462 - warp_offset; y2d = 512; x3d = 50 + warp_offset; y3d = 0; x4d = 462 - warp_offset; y4d = 0;																			  
	/// assign src points
	src_vertices[1] = Point(x3, y3); // 204, 447
	src_vertices[0] = Point(x4, y4); // 280, 447 230
	src_vertices[2] = Point(x2, y2);
	src_vertices[3] = Point(x1, y1); // 160, 512
	/// assign dst points
	dst_vertices[1] = Point(x3d, y3d);
	dst_vertices[0] = Point(x4d, y4d);
	dst_vertices[2] = Point(x2d, y2d);
	dst_vertices[3] = Point(x1d, y1d);
#endif
	
#ifdef VREP_CAM
	int warp_offset = 70;
	/// assign src points
	src_vertices[1] = Point(233, 280);
	src_vertices[0] = Point(277, 280);
	src_vertices[2] = Point(462, 512);
	src_vertices[3] = Point(50, 512);
	/// assign dst points
	dst_vertices[1] = Point(50 + warp_offset, 0);
	dst_vertices[0] = Point(462 - warp_offset, 0);
	dst_vertices[2] = Point(462 - warp_offset, 512);
	dst_vertices[3] = Point(50 + warp_offset, 512);
#endif
#ifdef DEBUGALL
    cout << "\t[DEBUGALL]\t[laneDetectionWEBOTS::get_bev_points] source vertices = " << src_vertices << "\tdestination vertices = " << dst_vertices << endl;
#endif
    	/// return result 
    	vertices[0] = src_vertices;
    	vertices[1] = dst_vertices;
    	return vertices;   
}

void laneDetectionWEBOTS::bev_transform(Mat& src, Mat& dst, int world_encode){
    /// get transform points 
    vector<vector<Point2f>> vertices = get_bev_points(world_encode); // 2D float points [SD] 
    /// calculate transform matrix
    Mat M = getPerspectiveTransform(vertices[0], vertices[1]);
    /// perform bev transformation 
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
}

void laneDetectionWEBOTS::bev_rev_transform(Mat& src, Mat& dst, int world_encode){
    /// get transform points 
    vector<vector<Point2f>> vertices = get_bev_points(world_encode);
    /// calculate reverse transform matrix
    Mat M = getPerspectiveTransform(vertices[1], vertices[0]);
    /// perform bev
    warpPerspective(dst, src, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
}

vector<vector<Point>> laneDetectionWEBOTS::sliding_window_lane_tracking(Mat& src){
	/// sliding window polyfit 
	Size shape = src.size();
	/// get bottom half of BEV image
	Mat img_bottom_half = src(Range(shape.height/2, shape.height), Range(0, shape.width));
	/// get histogram of bottom half
	Mat histogram;
	reduce(img_bottom_half, histogram, 0, REDUCE_SUM, CV_32S);
	/// midpoint of histogram
	int midpoint = shape.width/2;
	/// get left and right halves of histogram
	Point min_loc, max_loc;
	double minVal, maxVal; 
	Size histo = histogram.size();
	/// left half of histogram
	Mat hist_half = histogram(Range(0, histo.height), Range(0, histo.width/2));
	minMaxLoc(hist_half, &minVal, &maxVal, &min_loc, &max_loc);
	int leftx_base = max_loc.x;
	/// right half of histogram
	hist_half = histogram(Range(0, histo.height), Range(histo.width/2, histo.width));
	minMaxLoc(hist_half, &minVal, &maxVal, &min_loc, &max_loc);
	int rightx_base = max_loc.x + midpoint;

	/// Get x and y positions of all nonzero pixels in the image
	Mat nonzero;
	findNonZero(src, nonzero);

	/// Current positions to be updated for each window
	int leftx_current = leftx_base;
	int rightx_current = rightx_base;
	/// width of the windows
	int margin   = 65;
	/// minimum number of pixels needed to recenter window
	unsigned int minpix   = 100;
	/// number of sliding windows
	int nwindows = 8;
	/// Set height of sliding windows
	int window_height = shape.height/nwindows;   
	/// declare rectangle parameters
	int win_y_low, win_y_high, win_xleft_low, win_xleft_high, win_xright_low, win_xright_high;
	/// declare variables for mean calculation
	int total_good_left, counter_good_left, total_good_right, counter_good_right;
	/// Create vectors for points of left box and right box
	vector<Point> point_1_left(nwindows), point_1_right(nwindows);
	vector<Point> point_2_left(nwindows), point_2_right(nwindows);
	/// Create vectors to receive left and right lane pixel indices per window
	vector<int> good_left_inds;
	vector<int> good_right_inds;
	good_left_inds.reserve(nonzero.total());
	good_right_inds.reserve(nonzero.total());
	/// Create empty vectors to accumulate left and right lane pixel Points
	vector<Point> left_lane_inds, right_lane_inds;

	/// draw sliding windows and track the two lanes
	//for (int window = 0; window < nwindows; ++window){
	for (int window = 0; window < nwindows; ++window){
		/// set rectangle parameters
		win_y_low       = shape.height - (window + 1) * window_height;
		win_y_high      = shape.height - window * window_height;
		win_xleft_low   = leftx_current - margin;
		win_xleft_high  = leftx_current + margin;
		win_xright_low  = rightx_current - margin;
		win_xright_high = rightx_current + margin;

#ifdef DRAW_SLIDING_WINDOWS
		/// top left corner
		point_1_left[window]  = Point(win_xleft_low, win_y_high);
		point_1_right[window] = Point(win_xright_low, win_y_high);
		/// bottom right corner
		point_2_left[window]  = Point(win_xleft_high, win_y_low);
		point_2_right[window] = Point(win_xright_high, win_y_low);
		/// draw rectangles 
		rectangle(src, point_1_left[window], point_2_left[window], Scalar(255, 255, 255));
		rectangle(src, point_1_right[window], point_2_right[window], Scalar(255, 255, 255));
#endif

		/// initialize variables for mean calculation
		total_good_left    = 0;
		counter_good_left  = 0;
		total_good_right   = 0;
		counter_good_right = 0;

		/// Identify the nonzero pixels in x and y within the 2 rectangles
		for (int i = nonzero.total(); i > 0; --i) {
		    /// left rectangle
		    if ((nonzero.at<Point>(i).y >= win_y_low) && (nonzero.at<Point>(i).y < win_y_high) && 
		        (nonzero.at<Point>(i).x >= win_xleft_low) && (nonzero.at<Point>(i).x < win_xleft_high)){
		            /// Append candidate point.x to the current window vector
		            /// good_left_inds[i] = nonzero.at<Point>(i).x;
		            good_left_inds.push_back(nonzero.at<Point>(i).x);
		            /// mean calculation
		            total_good_left += nonzero.at<Point>(i).x;
		            counter_good_left++;
		            /// Append candidate point to the accumulation vector
		            left_lane_inds.push_back(nonzero.at<Point>(i));
		        }
		    /// right rectangle
		    if ((nonzero.at<Point>(i).y >= win_y_low) && (nonzero.at<Point>(i).y < win_y_high) && 
		        (nonzero.at<Point>(i).x >= win_xright_low) && (nonzero.at<Point>(i).x < win_xright_high)){
		            /// Append candidate point.x to the current window vector
		            /// good_right_inds[i] = nonzero.at<Point>(i).x;
		            good_right_inds.push_back(nonzero.at<Point>(i).x);
		            /// mean calculation
		            total_good_right += nonzero.at<Point>(i).x;
		            counter_good_right++;
		            /// Append candidate point to the accumulation vector
		            right_lane_inds.push_back(nonzero.at<Point>(i));
		        }
		}
		/// If you found > minpix pixels, recenter next window on their mean position
		if ((good_left_inds.size() > minpix) && counter_good_left ){
		    leftx_current  = total_good_left / counter_good_left;
		}
		if ((good_right_inds.size() > minpix) && counter_good_right ){
		    rightx_current = total_good_right / counter_good_right;
		}
	}   

    	/// return identified points for both lanes
	vector<vector<Point>> lanes(2);
	lanes[0] = left_lane_inds;
	lanes[1] = right_lane_inds;
	return lanes;
}

long double laneDetectionWEBOTS::calculate_lateral_deviation(vector<Point> left_lane_inds, vector<Point> right_lane_inds,float ref_tune_hardcoded){
	///  Extract left and right line pixel positions
	vector<float> leftx(left_lane_inds.size()), lefty(left_lane_inds.size());
	vector<float> rightx(right_lane_inds.size()), righty(right_lane_inds.size());

	for (unsigned int i = 0; i < left_lane_inds.size(); ++i){
		leftx[i] = left_lane_inds[i].x;
		lefty[i] = left_lane_inds[i].y;
	}
	for (unsigned int i = 0; i < right_lane_inds.size(); ++i){
		rightx[i] = right_lane_inds[i].x;
		righty[i] = right_lane_inds[i].y;
	}
	/// Fit a second order polynomial to left and right lane positions
	//vector<float> left_fit = mathalgo::polyfitparallel( lefty, leftx, 2 );
	//vector<float> right_fit = mathalgo::polyfitparallel( righty, rightx, 2 );
	vector<float> left_fit = mathalgo::polyfit( lefty, leftx, 2 );
	vector<float> right_fit = mathalgo::polyfit( righty, rightx, 2 );

	/// calculate lateral deviation yL at look-ahead distance LL = 5.5 meters
	//vector<long double> vec1{483.0L}, vec2{512.0L}; // changed vec2(512.0L) [SD]
	vector<float> vec2{512.0L}; 
	vector<float> scale_leftx = mathalgo::polyval(left_fit, vec2); 
    	vector<float> scale_rightx = mathalgo::polyval(right_fit, vec2); 
	float scale = 3.25 / (scale_rightx[0] - scale_leftx[0]);
	//cout << scale << endl;
	
	float ref, ll_pixel;
#ifdef WEBOTS_CAM
	ll_pixel =  5.5 *  WEBOTS_YPIXEL_PER_METRE;
#endif

	vector<float> vec1{ll_pixel}; 
	vector<float> yL_leftx = mathalgo::polyval(left_fit, vec1); 
    	vector<float> yL_rightx = mathalgo::polyval(right_fit, vec1);
	
#ifdef WEBOTS_CAM
	if ((yL_leftx[0] > WEBOTS_LEFT_THRESOLD) && (yL_rightx[0] > WEBOTS_RIGHT_THRESOLD)) ref = x1d + (((x2d - x1d)/(x2 - x1))*(256 - x1)) - ref_tune_hardcoded;
	else ref = x1d + (((x2d - x1d)/(x2 - x1))*(256 - x1));
#endif
	
	long double yL = (ref - (yL_leftx[0] + yL_rightx[0]) / 2) * scale;							
	//yL[1] = (ref - (scale_leftx[0] + scale_rightx[0]) / 2) * scale;

#ifdef DEBUGALL
	cout << "\t[DEBUGALL]\t[laneDetectionWEBOTS::calculate_lateral_deviation] yL = " << yL << "\tRef = " << ref << endl;
#endif

    	return yL; // in meters
}

void laneDetectionWEBOTS::lane_identification(vector<vector<Point>> lanes, Mat& src, Mat& img_roi, 
										Mat& img_warped, Mat& img_detected_lanes, 
										Mat& draw_lines, Mat& diff_src_rebev, Mat& img_rev_warped, int world_encode) {
    /// reverting back to original image
    draw_lines = img_warped.clone();

    /// draw circes iso lines;
    for (unsigned int i = 0; i < lanes[0].size(); ++i){
        circle(draw_lines, lanes[0][i], 1, Scalar(0, 0, 255), 1, 8); //Scalar (b, g, r) [SD]
    }
    for (unsigned int i = 0; i < lanes[1].size(); ++i){
        circle(draw_lines, lanes[1][i], 1, Scalar(0, 0, 255), 1, 8);
    }

    /// get reverse bev on bev with drawed detected lines 
    bev_rev_transform(img_rev_warped, draw_lines, world_encode);
    /// reverse bev transform on bev of src image 
    bev_rev_transform(img_roi, img_warped, world_encode);
    /// substract: diff_src_rebev = src - img_roi 
    absdiff(src, img_roi, diff_src_rebev);
    /// combine: diff_src_rebev + img_rev_warped
    bitwise_or(diff_src_rebev, img_rev_warped, img_detected_lanes);    
}


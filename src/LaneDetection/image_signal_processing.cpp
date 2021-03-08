/** @file image_signal_processing.cpp
 *  @brief source file for ISP pipeline
 *
 *  Name            :   image_signal_processing.cpp
 *
 *  Authors         :   Sayandip De (sayandip.de@tue.nl)
 *			Sajid Mohamed (s.mohamed@tue.nl)
 *
 *  Date            :   March 26, 2020
 *
 *  Function        :   run different approximate ISP versions
 *
 *  History         :
 *      26-03-20    :   Initial version.
 *						Code is modified from https://github.com/mbuckler/ReversiblePipeline [written by Mark Buckler]. 
 *
 */

#include "image_signal_processing.hpp"

using namespace cv;
using namespace Halide;
using namespace Halide::Tools;

imageSignalProcessing::imageSignalProcessing(){}  //!< constructor
imageSignalProcessing::~imageSignalProcessing(){} //!< destructor

Mat imageSignalProcessing::approximate_pipeline(Mat the_img_input, int the_pipe_version) { 

	using namespace std;

	Mat img_rev, img_fwd, v0_rev, v0_fwd; //!< img_rev: image after applying approximated reverse pipeline 
					//!< img_fwd: image after applying approximated forward ISP pipeline  
					//!< v0_rev: image after applying reverse pipeline without approximation
					//!< v0_fwd: image after applying forward ISP pipeline without approximation

	//// Run ISP pipeline (reverse image from VREP/webots to obtain RAW and forward ISP pipeline to obtain *.jpg) for the original image (v0 in papers)
	v0_rev = run_pipeline(false, the_img_input, 0); //!< v0_rev: image after applying reverse pipeline without approximation
	v0_fwd = run_pipeline(true, v0_rev, 0);	  //!< v0_fwd: image after applying forward ISP pipeline without approximation

	if (the_pipe_version != 0){ //!< image needs approximation
	//// Run reverse pipeline for the original image from the simulator to obtain RAW image
	img_rev = run_pipeline(false, v0_fwd, the_pipe_version);
	if (the_pipe_version == 7) return img_rev;

	//// Run forward pipeline for the RAW image
	img_fwd = run_pipeline(true, img_rev, the_pipe_version);
	} else { //!< no approximation required
	img_rev = v0_rev;
	img_fwd = v0_fwd;
	}

	//// save_images
	imwrite(out_string+"/output_rev.png", img_rev);
	imwrite(out_string+"/output_fwd_v"+to_string(the_pipe_version)+".png", img_fwd);

	return img_fwd;
}

//// Reversible pipeline function
Mat imageSignalProcessing::run_pipeline(bool direction, Mat the_img_in, int the_pipe_version) {
  using namespace std;  		
  ///////////////////////////////////////////////////////////////
  /// Input Image Parameters
  ///////////////////////////////////////////////////////////////
   Halide::Buffer<uint8_t> input1;
   Halide::Runtime::Buffer<uint8_t> input;
  /// convert input Mat to Image
  if (the_pipe_version == 8 && direction){ //!< if the approximate_pipeline_version == 8
	/// Convert the input Matrix image to RGB image in Halide::Buffer<uint8_t> format
	input1 = Mat2Image <Halide::Buffer<uint8_t>, uint8_t> (&the_img_in);
	Mat mat_input, mat_output;
	int width  = input1.width();
	int height = input1.height();
	/// make_scale the input1 image
	Func scale              = make_scale        ( &input1);
	/// scale.realize the opencv_in_image image to  input image's (width, height, 3)
	Buffer<float> opencv_in_image = scale.realize(width, height, 3);
	/// Convert scaled RGB image to BGR Matrix
	Mat opencv_in_mat = Image2Mat <Buffer<float>,float,323> (&opencv_in_image); // 323: OutMat(height,width,CV_32FC3);
	/// OpenCV remosaic the BGR Matrix 
	OpenCV_remosaic(&opencv_in_mat);
	/// Convert Matrix to RGB image
	Halide::Runtime::Buffer<float> opencv_out = Mat2Image <Halide::Runtime::Buffer<float>,float> (&opencv_in_mat);
	/// initialise output_n in Halide::Runtime::Buffer<uint8_t> format using input1 image details
	Halide::Runtime::Buffer<uint8_t> output_n(input1.width(), input1.height(), input1.channels());
	/// auto_schedule_dem_fwd()
	auto_schedule_dem_fwd(opencv_out, output_n);
	/// Convert output_n to Matrix form
	mat_output = Image2Mat <Halide::Runtime::Buffer<uint8_t>,uint8_t,83> (&output_n); // 83: OutMat(height,width,CV_8UC3);
	/// Write image to file
	imwrite("out_imgs/img_v8.png", mat_output);
	mat_input = imread("out_imgs/img_v8.png", IMREAD_COLOR);
	/// Convert image Matrix to input image forcompression/perception stage
	input = Mat2Image <Halide::Runtime::Buffer<uint8_t>,uint8_t> (&mat_input); 
  }
  else{
	input = Mat2Image <Halide::Runtime::Buffer<uint8_t>,uint8_t> (&the_img_in);  
  }

  ///////////////////////////////////////////////////////////////////////////////////////
  /// Import and format model data: cpp -> halide format

  /// Declare model parameters
  vector<vector<float>> Ts, Tw, TsTw;
  vector<vector<float>> ctrl_pts, weights, coefs;
  vector<vector<float>> rev_tone;

  /// Load model parameters from file
  /// NOTE: Ts, Tw, and TsTw read only forward data
  /// ctrl_pts, weights, and coefs are either forward or backward
  /// tone mapping is always backward
  /// This is due to the the camera model format
  ///////////////////////////////////////////////////////////////
  /// Camera Model Parameters
  ///////////////////////////////////////////////////////////////

    /// White balance index (select white balance from transform file)
  /// The first white balance in the file has a wb_index of 1
  /// For more information on model format see the readme
  int wb_index = 3; /// Actual choice = Given no. + 1

  /// Number of control points
  const int num_ctrl_pts = 3702;

  Ts        = get_Ts       (&cam_model_path[0]);
  Tw        = get_Tw       (&cam_model_path[0], wb_index);
  TsTw      = get_TsTw     (&cam_model_path[0], wb_index);
  ctrl_pts  = get_ctrl_pts (&cam_model_path[0], num_ctrl_pts, direction);
  weights   = get_weights  (&cam_model_path[0], num_ctrl_pts, direction);
  coefs     = get_coefs    (&cam_model_path[0], num_ctrl_pts, direction);
  rev_tone  = get_rev_tone (&cam_model_path[0]);
  
  /// Take the transpose of the color map and white balance transform for later use
  vector<vector<float>> TsTw_tran = transpose_mat (TsTw); /// source of segmentation error

  /// If we are performing a backward implementation of the pipeline, 
  /// take the inverse of TsTw_tran
  if (direction == 0) { 
    TsTw_tran = inv_3x3mat(TsTw_tran);
  }

  using namespace Halide;
  using namespace Halide::Tools;

  /// Convert control points to a Halide image
  int width  = ctrl_pts[0].size();
  int length = ctrl_pts.size();

  Halide::Runtime::Buffer<float> ctrl_pts_h(width,length);
  for (int y=0; y<length; y++) {
    for (int x=0; x<width; x++) {
      ctrl_pts_h(x,y) = ctrl_pts[y][x];
    }
  }
  /// Convert weights to a Halide image
  width  = weights[0].size();
  length = weights.size();

  Halide::Runtime::Buffer<float> weights_h(width,length);
  for (int y=0; y<length; y++) {
    for (int x=0; x<width; x++) {
      weights_h(x,y) = weights[y][x];
    }
  }
  /// Convert the reverse tone mapping function to a Halide image
  width  = 3;
  length = 256;
  Halide::Runtime::Buffer<float> rev_tone_h(width,length);
  for (int y=0; y<length; y++) {
    for (int x=0; x<width; x++) {
      rev_tone_h(x,y) = rev_tone[y][x];
    }
  }
 

  /// Convert the TsTw_tran to a Halide image
  width  = 3;
  length = 3;
  Halide::Runtime::Buffer<float> TsTw_tran_h(width,length);
  for (int y=0; y<length; y++) {
    for (int x=0; x<width; x++) {
      TsTw_tran_h(x,y) = TsTw_tran[x][y];
    }
  }

  /// Convert the coefs to a Halide image
  width  = 4;
  length = 3; //4 [SD]
  Halide::Runtime::Buffer<float> coefs_h(width,length);
  for (int y=0; y<length; y++) {
    for (int x=0; x<width; x++) {
      coefs_h(x,y) = coefs[x][y];
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////////
  /// Declare image handle variables
  Var x, y, c;

  ////////////////////////////////////////////////////////////////////////
  /// run the generator

  Halide::Runtime::Buffer<uint8_t> output(input.width(), input.height(), input.channels());

  if (direction == 1) {
    switch(the_pipe_version){
      case 0: /// full rev + fwd                   skip 0 stages
        auto_schedule_true_fwd_v0(input, ctrl_pts_h, weights_h, rev_tone_h, TsTw_tran_h, coefs_h, output);
        break;
      case 1: /// rev + fwd_transform_tonemap      skip 1 stage
        auto_schedule_true_fwd_v1(input, ctrl_pts_h, weights_h, rev_tone_h, TsTw_tran_h, coefs_h, output);
        break;      
      case 2: /// rev + fwd_transform_gamut        skip 1 stage
       auto_schedule_true_fwd_v2(input, ctrl_pts_h, weights_h, rev_tone_h, TsTw_tran_h, coefs_h, output);
        break;
      case 3: /// rev + fwd_gamut_tonemap          skip 1 stage
        auto_schedule_true_fwd_v3(input, ctrl_pts_h, weights_h, rev_tone_h, TsTw_tran_h, coefs_h, output);
        break;
      case 4: /// rev + fwd_transform              skip 2 stages
        auto_schedule_true_fwd_v4(input, ctrl_pts_h, weights_h, rev_tone_h, TsTw_tran_h, coefs_h, output);
        break;      
      case 5: /// rev + fwd_gamut                  skip 2 stages
        auto_schedule_true_fwd_v5(input, ctrl_pts_h, weights_h, rev_tone_h, TsTw_tran_h, coefs_h, output);
        break;      
      case 6: /// rev + fwd_tonemap                skip 2 stages
        auto_schedule_true_fwd_v6(input, ctrl_pts_h, weights_h, rev_tone_h, TsTw_tran_h, coefs_h, output);
        break;
      case 8: /// skip denoise
	{auto_schedule_true_fwd_v0(input, ctrl_pts_h, weights_h, rev_tone_h, TsTw_tran_h, coefs_h, output);
		/// cout << "here3.9" << endl;
	 break; }
      default: /// should not happen
        cout << "Approximate pipeline version not defined. Default branch taken, pls check.\n";
        break;
    }/// switch      
  } else {
	 auto_schedule_true_rev(input, ctrl_pts_h, weights_h, rev_tone_h, TsTw_tran_h, coefs_h, output); 
  }

  ///////////////////////////////////////////////////////////////////////
  /// convert Image to Mat and return

  Mat img_out;
  img_out = Image2Mat <Halide::Runtime::Buffer<uint8_t>,uint8_t,83> (&output); //83: CV_8UC3 (DEFAULT for OutMat)
  return img_out;
}

Halide::Func imageSignalProcessing::make_scale( Buffer<uint8_t> *in_img ) {

  Var x, y, c;
  /// Cast input to float and scale from 8 bit 0-255 range to 0-1 range
  Func scale("scale");
    scale(x,y,c) = cast<float>( (*in_img)(x,y,c) ) / 255;
  return scale;
}

void imageSignalProcessing::OpenCV_remosaic (Mat *InMat ) {

  vector<Mat> three_channels;
  cv::split((*InMat), three_channels);

  /// Re-mosaic aka re-bayer the image
  /// B G
  /// G R
  /// Note: OpenCV stores as BGR not RGB
  for (int y=0; y<(*InMat).rows; y++) {
    for (int x=0; x<(*InMat).cols; x++) {
      /// If an even row
      if ( y%2 == 0 ) {
        /// If an even column
        if ( x%2 == 0 ) {
          /// Green pixel, remove blue and red
          three_channels[0].at<float>(y,x) = 0; 
          three_channels[2].at<float>(y,x) = 0;
          /// Also divide the green by half to account
          /// for interpolation reversal
          three_channels[1].at<float>(y,x) = 
            three_channels[1].at<float>(y,x) / 2;
        }
        /// If an odd column
        else {
          /// Red pixel, remove blue and green
          three_channels[0].at<float>(y,x) = 0;
          three_channels[1].at<float>(y,x) = 0;
        }
      }
      /// If an odd row
      else {
        /// If an even column
        if ( x%2 == 0 ) {
          /// Blue pixel, remove red and green
          three_channels[2].at<float>(y,x) = 0;
          three_channels[1].at<float>(y,x) = 0;
        }
        /// If an odd column
        else {
          /// Green pixel, remove blue and red
          three_channels[0].at<float>(y,x) = 0;
          three_channels[2].at<float>(y,x) = 0;
          /// Also divide the green by half to account
          /// for interpolation reversal
          three_channels[1].at<float>(y,x) = 
            three_channels[1].at<float>(y,x) / 2;
        }
      }
    }
  }
  cv::merge(three_channels, *InMat);

}

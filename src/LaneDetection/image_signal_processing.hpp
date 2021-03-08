/** @file image_signal_processing.hpp
 *  @brief header file for ISP pipeline
*/
#ifndef LANEDETECTION_IMAGE_SIGNAL_PROCESSING_H_
#define LANEDETECTION_IMAGE_SIGNAL_PROCESSING_H_

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <math.h>

/// Reversible pipeline include files
#include "auto_schedule_true_rev.h"

/// Forward pipeline include files
#include "auto_schedule_true_fwd_v0.h"
#include "auto_schedule_true_fwd_v1.h"
#include "auto_schedule_true_fwd_v2.h"
#include "auto_schedule_true_fwd_v3.h"
#include "auto_schedule_true_fwd_v4.h"
#include "auto_schedule_true_fwd_v5.h"
#include "auto_schedule_true_fwd_v6.h"

/// Generated Profiling include files
#include "auto_schedule_dem_fwd.h"

/// Halide include files
#include "Halide.h"
#include "halide_benchmark.h"
#include "halide_image_io.h"
#include "halide_image.h"

/// #######################
#include "LoadCamModel.h"
#include "MatrixOps.h"

#include "paths.hpp"

/// @brief Class for image signal processing (ISP) pipeline. Inherits the base class pathsIMACS to load the paths to image.
class imageSignalProcessing : public pathsIMACS {
private:
    /**  @brief C++ implementation of ISP pipeline
           Computes the forward or reverse ISP pipeline based on the function parameter 'direction'.
         		forward ISP: converts RAW image to compressed RGB image
         		reverse ISP: converts compressed image (typically obtained from simulator) to RAW format
         @param[in] direction            0: reverse pipeline, 1: forward ISP
         @param[in] the_img_in           Input image
         @param[in] the_pipe_version     The approximate pipeline version. See IEEE Access paper [de2020access] for details
         @return    the image after applying the ISP pipeline
      */
    cv::Mat run_pipeline(bool direction, cv::Mat the_img_in, int the_pipe_version);

    /**  @brief C++ template implementation to convert from interleaved BGR memory storage to planar RGB memory storage
         @param[in] InMat      pointer to Matrix format BGR image in memory
         @return    RGB image in IMG_FORMAT with each pixel of PIXEL_TYPE
      */    
    template <typename IMG_FORMAT,typename PIXEL_TYPE>
    IMG_FORMAT Mat2Image( cv::Mat *InMat );

    /** @brief C++ template implementation to convert from planar RGB memory storage to interleaved BGR memory storagee
        @param[in] InImage      pointer to RGB image in IMG_FORMAT with each pixel of PIXEL_TYPE in memory
        @return    BGR matrix image of type cv::Mat
     */ 
    template <typename IMG_FORMAT,typename PIXEL_TYPE,int OUT_MAT_TYPE>
    cv::Mat Image2Mat(IMG_FORMAT *InImage );

    /** @brief C++ implementation to make scale of the input image. Cast input to float and scale from 8 bit 0-255 range to 0-1 range.
        @param[in] in_img     pointer to input image in Halide::Buffer<uint8_t> format 
        @return    Halide::Func scale
     */ 
    Halide::Func make_scale(Halide::Buffer<uint8_t> *in_img );

    /**  @brief C++ implementation of remosaic (re-Bayer the image in Matrix form)
         @param[in,out] InMat      pointer to Matrix format BGR image in memory
    */
    void OpenCV_remosaic (cv::Mat *InMat );
public:
    /// constructor
    imageSignalProcessing();
    /// destructor
    ~imageSignalProcessing();
    /**  @brief C++ implementation of approximate pipeline for IMACS
           For the captured image from the simulator, does the reverse ISP to RAW image and then forward ISP based on the version.
         forward ISP: converts RAW image to compressed RGB image
         reverse ISP: converts compressed image (typically obtained from simulator) to RAW format
         @note
         @param[in] the_img_input        The image captured from the simulator (VREP/WEBOTS)
         @param[in] the_pipe_version     The approximate pipeline version. See IEEE Access paper [de2020access] for details
         @return    the image after applying the reverse ISP and forward ISP pipeline
      */
    cv::Mat approximate_pipeline(cv::Mat the_img_input, int the_pipe_version);
};

/// template function declaration for Image2Mat
template <typename IMG_FORMAT,typename PIXEL_TYPE,int OUT_MAT_TYPE>
cv::Mat imageSignalProcessing::Image2Mat(IMG_FORMAT *InImage ) {
  
  int height = (*InImage).height();
  int width  = (*InImage).width();
  cv::Mat OutMat; //!< initialise OutMat so that it is in the scope of the function
  if (OUT_MAT_TYPE==323) //!< hardcoded value; If you want to use other formats, edit here
  	OutMat.create(height,width,CV_32FC3); //!< Turn OutMat into CV_32FC3 --> 3 channel (complex) floating-point array. The old content will be deallocated
  else
  	OutMat.create(height,width,CV_8UC3);  //!< Turn OutMat into CV_8UC3 --> 3 channel (complex) 8-bit array. The old content will be deallocated
  vector<cv::Mat> three_channels;
  cv::split(OutMat, three_channels);

  /// Convert from planar RGB memory storage to interleaved BGR memory storage
  for (int y=0; y<height; y++) {
    for (int x=0; x<width; x++) {
      /// Blue channel
      three_channels[0].at<PIXEL_TYPE>(y,x) = (*InImage)(x,y,2);
      /// Green channel
      three_channels[1].at<PIXEL_TYPE>(y,x) = (*InImage)(x,y,1);
      /// Red channel
      three_channels[2].at<PIXEL_TYPE>(y,x) = (*InImage)(x,y,0);
    }
  }

  cv::merge(three_channels, OutMat);

  return OutMat;
}

/// template function declaration for Mat2Image
template <typename IMG_FORMAT,typename PIXEL_TYPE>
IMG_FORMAT imageSignalProcessing::Mat2Image( cv::Mat *InMat ) {
	
  int height = (*InMat).rows;
  int width  = (*InMat).cols;
  IMG_FORMAT OutImage(width,height,3);
  vector<cv::Mat> three_channels;

  /// Split matrix into three channels
  cv::split((*InMat), three_channels);

  /// Convert from interleaved BGR memory storage to planar RGB memory storage
  for (int y=0; y<height; y++) {
    for (int x=0; x<width; x++) {
      /// Convert Matrix Blue channel to RGB image's Blue channel
      OutImage(x,y,2) = three_channels[0].at<PIXEL_TYPE>(y,x);
      /// Convert Matrix Green channel to RGB image's Green channel
      OutImage(x,y,1) = three_channels[1].at<PIXEL_TYPE>(y,x);
      /// Convert Matrix Red channel to RGB image's Red channel
      OutImage(x,y,0) = three_channels[2].at<PIXEL_TYPE>(y,x);
    }
  }
  return OutImage;
}

#endif

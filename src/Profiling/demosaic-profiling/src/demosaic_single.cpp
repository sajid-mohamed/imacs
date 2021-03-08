/** @file demosaic_single.cpp
 *  @brief run demosaic-cpu profiling with single image workloads
 *  @description

 *  Copyright (c) 2020 sayandipde
 *  Eindhoven University of Technology
 *  Eindhoven, The Netherlands
 *
 *  Name            :   image_signal_processing.cpp
 *
 *  Authors         :   Sayandip De (sayandip.de@tue.nl)
 *			Sajid Mohamed (s.mohamed@tue.nl)
 *
 *  Date            :   March 26, 2020
 *
 *  Function        :   run demosaic-cpu profiling with single image workloads
 *
 *  History         :
 *      26-03-20    :   Initial version.
 *			Code is modified from https://github.com/mbuckler/ReversiblePipeline [written by Mark Buckler]. 
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
 
// Halide
#include "auto_schedule_dem_fwd.h"
#include "Halide.h"
//#include "halide_benchmark.h"
#include "halide_image_io.h"
#include "halide_image.h"
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <math.h>
//#include <chrono>
#ifdef PROFILEWITHCHRONO
#include "../my_profiler.hpp" 
#endif

#ifdef PROFILEWITHVALGRIND
#include "callgrind.h" 
#endif


// Pipeline V79
// 
// Test type: 
// Debug
// 
// Stages:
// Remosaic, DemosInterp

using namespace cv;
using namespace std;
//using namespace std::chrono; 
using namespace Halide;
using namespace Halide::Tools;

#ifdef PROFILEWITHCHRONO
	template<class Container>
	std::ostream& write_container(const Container& c, std::ostream& out, string pipeversion, char delimiter = '\n')
	{
		out << pipeversion;
		out << delimiter;
		bool write_sep = false;
		for (const auto& e: c) {
			if (write_sep)
				out << delimiter;
			else
				write_sep = true;
			out << e;
		}
		return out;
	}
#endif

// fucntion prototypes
void OpenCV_remosaic (Mat *InMat );
Halide::Runtime::Buffer<float> Mat2Image( Mat *InMat );
Mat Image2Mat( Buffer<float> *InImage );
Func make_scale( Buffer<uint8_t> *in_img );

int main(int argc, char **argv){
  // Inform user of usage method
  if ( argc != 3 )
  {
      printf("usage: \n./convert path/to/in/image out/image/dir\n");
      return -1;
  }

  // Input image (path and name)
  const char * in_img_path = argv[1];

  // Output image (just path)
  const char * out_path    = argv[2];

  ///////////////////////////////////////////////////////////////////////////////////////
  // Import and format input image

  // Declare image handle variables
  Var x, y, c;

  // Load input image 
  Image<uint8_t> input = load_image(in_img_path);

  ///////////////////////////////////////////////////////////////////////////////////////
  // Reverse Camera pipeline

  int width  = input.width();
  int height = input.height();
  string out_string;

  // Scale to 0-1 range and represent in floating point
  Func scale              = make_scale        ( &input);

  Buffer<float> opencv_in_image = scale.realize(width, height, 3);

  Mat opencv_in_mat = Image2Mat(&opencv_in_image);

  // open cv
  OpenCV_remosaic(&opencv_in_mat);

  Halide::Runtime::Buffer<float> opencv_out = Mat2Image(&opencv_in_mat);
  
  //imwrite(std::string(out_path)+"raw.png", opencv_in_mat);

  ///////////////////////////////////////////////////////////////////////////////////////
  // Forward Camera pipeline 
  // Use AOT compilation
  Halide::Runtime::Buffer<uint8_t> output(input.width(), input.height(), input.channels());


  //////////////////////////////////////////////////////////////////////////////////////////////////
  // Timing code

  vector<vector<double>> wc_avg_bc_tuples;
    
#ifdef PROFILEWITHCHRONO
  wc_avg_bc_tuples = do_benchmarking( [&]() { auto_schedule_dem_fwd(opencv_out, output); } ); // size = [1 500]

  /*for (int i = 0; i < wc_avg_bc_tuples.size(); i++){
    for (int j = 0; j < wc_avg_bc_tuples[i].size(); j++){
      cout << wc_avg_bc_tuples[i][j] << " ";
    }
    cout << "\n";
  }*/
  cout << "wc_avg_bc_tuples [size] = " << wc_avg_bc_tuples.size() << " " << wc_avg_bc_tuples[0].size() << endl;

  out_string = "chrono/runtime_demos_single.csv";
  std::ofstream outfile(out_string);
  write_container(wc_avg_bc_tuples[0], outfile, "v0");

////////////////////////////////////////////////////////////////////
  // Save the output
  save_image(output, (std::string(out_path)+"output.png").c_str());
#endif

#ifdef PROFILEWITHVALGRIND
	CALLGRIND_TOGGLE_COLLECT;
	auto_schedule_dem_fwd(opencv_out, output);
	CALLGRIND_TOGGLE_COLLECT;
	CALLGRIND_DUMP_STATS;
#endif
  return 0;
}

Mat Image2Mat( Buffer<float> *InImage ) {
  int height = (*InImage).height();
  int width  = (*InImage).width();
  Mat OutMat(height,width,CV_32FC3);
  vector<Mat> three_channels;
  split(OutMat, three_channels);

  // Convert from planar RGB memory storage to interleaved BGR memory storage
  for (int y=0; y<height; y++) {
    for (int x=0; x<width; x++) {
      // Blue channel
      three_channels[0].at<float>(y,x) = (*InImage)(x,y,2);
      // Green channel
      three_channels[1].at<float>(y,x) = (*InImage)(x,y,1);
      // Red channel
      three_channels[2].at<float>(y,x) = (*InImage)(x,y,0);
    }
  }

  merge(three_channels, OutMat);

  return OutMat;
}

Halide::Runtime::Buffer<float> Mat2Image( Mat *InMat ) {
  int height = (*InMat).rows;
  int width  = (*InMat).cols;
  Halide::Runtime::Buffer<float> OutImage(width,height,3);
  vector<Mat> three_channels;
  split((*InMat), three_channels);

  // Convert from interleaved BGR memory storage to planar RGB memory storage
  for (int y=0; y<height; y++) {
    for (int x=0; x<width; x++) {
      // Blue channel
      OutImage(x,y,2) = three_channels[0].at<float>(y,x);
      // Green channel
      OutImage(x,y,1) = three_channels[1].at<float>(y,x);
      // Red channel
      OutImage(x,y,0) = three_channels[2].at<float>(y,x);
    }
  }

  return OutImage;
}

void OpenCV_remosaic (Mat *InMat ) {

  vector<Mat> three_channels;
  cv::split((*InMat), three_channels);

  // Re-mosaic aka re-bayer the image
  // B G
  // G R

  // Note: OpenCV stores as BGR not RGB
  for (int y=0; y<(*InMat).rows; y++) {
    for (int x=0; x<(*InMat).cols; x++) {
      // If an even row
      if ( y%2 == 0 ) {
        // If an even column
        if ( x%2 == 0 ) {
          // Green pixel, remove blue and red
          three_channels[0].at<float>(y,x) = 0; 
          three_channels[2].at<float>(y,x) = 0;
          // Also divide the green by half to account
          // for interpolation reversal
          three_channels[1].at<float>(y,x) = 
            three_channels[1].at<float>(y,x) / 2;
        }
        // If an odd column
        else {
          // Red pixel, remove blue and green
          three_channels[0].at<float>(y,x) = 0;
          three_channels[1].at<float>(y,x) = 0;
        }
      }
      // If an odd row
      else {
        // If an even column
        if ( x%2 == 0 ) {
          // Blue pixel, remove red and green
          three_channels[2].at<float>(y,x) = 0;
          three_channels[1].at<float>(y,x) = 0;
        }
        // If an odd column
        else {
          // Green pixel, remove blue and red
          three_channels[0].at<float>(y,x) = 0;
          three_channels[2].at<float>(y,x) = 0;
          // Also divide the green by half to account
          // for interpolation reversal
          three_channels[1].at<float>(y,x) = 
            three_channels[1].at<float>(y,x) / 2;
        }
      }
    }
  }
  cv::merge(three_channels, *InMat);

}


Func make_scale( Buffer<uint8_t> *in_img ) {
  Var x, y, c;
  // Cast input to float and scale from 8 bit 0-255 range to 0-1 range
  Func scale("scale");
    scale(x,y,c) = cast<float>( (*in_img)(x,y,c) ) / 255;
  return scale;
}


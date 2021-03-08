/** @file LoadCamModel.h
 *  @brief The header file to load camera model
 */
#include <vector>
#include <string>
#include <cstring>
#include <fstream>
#include <sstream>
#include <iostream>

using namespace std;

/// @brief Get color space transform
vector<vector<float>> get_Ts(char* cam_model_path);

/// @brief Get white balance transform
vector<vector<float>> get_Tw(char* cam_model_path, int wb_index);

/// @brief Get combined transforms for checking
vector<vector<float>> get_TsTw(char* cam_model_path, int wb_index);

/// @brief Get control points
vector<vector<float>> get_ctrl_pts(char* cam_model_path, int num_cntrl_pts, bool direction);

/// @brief Get weights
vector<vector<float>> get_weights(char* cam_model_path, int num_cntrl_pts, bool direction);

/// @brief Get coeficients 
vector<vector<float>> get_coefs(char* cam_model_path, int num_cntrl_pts, bool direction);

/// @brief Get reverse tone mapping
vector<vector<float>> get_rev_tone(char* cam_model_path);

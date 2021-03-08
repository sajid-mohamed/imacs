/** @file utils.hpp
 *  @brief header file for holding some utililty functions
 */
#ifndef UTILS_H_
#define UTILS_H_

#include <ctime>
#include <sstream>
#include <iomanip>
#include <fstream>

/// @brief Class for utils
class utils{
public:
	/// constructor
	utils(){}
	/// destructor
	~utils(){}
	
    	/**  @brief C++ implementation to get current timestamp in [YYYY-MM-DD HH:MM:SS] format
      	*/    
	std::string get_timestamp();
	
    	/**  @brief C++ implementation to get current timestamp in [YYYY_MM_DD-HHMM] format
      	*/    
	std::string get_timestamp_filename();
};

#endif

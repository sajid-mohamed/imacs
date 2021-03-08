/** @file utils.cpp
 *  @brief source file for holding some utililty functions
 */
#include <ctime>
#include <sstream>
#include <iomanip>
#include <fstream>
#include "utils.hpp"

std::string utils::get_timestamp(){
    time_t now = time(0);
    tm *ltm = localtime(&now);
    std::ostringstream oss;
    oss << "["
        << 1900 + ltm->tm_year 
        << "-"
        << std::setfill('0') << std::setw(2) << 1 + ltm->tm_mon 
        << "-"
        << std::setfill('0') << std::setw(2) << ltm->tm_mday 
        << " "  
        << std::setfill('0') << std::setw(2) << ltm->tm_hour 
        << ":"
        << std::setfill('0') << std::setw(2) << ltm->tm_min
        << ":"
        << std::setfill('0') << std::setw(2) << ltm->tm_sec
        << "]";
    std::string timestamp = oss.str();   
    return timestamp;
}

std::string utils::get_timestamp_filename(){
    time_t now = time(0);
    tm *ltm = localtime(&now);
    std::ostringstream oss;
    oss << "["
        << 1900 + ltm->tm_year 
        << "_"
        << std::setfill('0') << std::setw(2) << 1 + ltm->tm_mon 
        << "_"
        << std::setfill('0') << std::setw(2) << ltm->tm_mday 
        << "-"  
        << std::setfill('0') << std::setw(2) << ltm->tm_hour
        << std::setfill('0') << std::setw(2) << ltm->tm_min
        << "]";
    std::string timestamp = oss.str();   
    return timestamp;
}


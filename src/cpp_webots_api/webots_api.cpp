/** @file webots_api.cpp
 *  @brief The source file for webots API ("extern controller") when using IMACS framework using WEBOTS simulator
 */
#include "webots_api.hpp"

///--- convert image_2_Mat ---//
Mat webotsAPI::webots_img_2_Mat(Camera *camera, const unsigned char *image)
{
	/// input: 3 color image (RGB) 
	const int width  = camera_width;
	const int height = camera_height;
	Mat out_Mat(height, width, CV_8UC3);
 	vector<Mat> three_channels;
 	split(out_Mat, three_channels);		
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			/// Get Blue channel from camera image and store it to first channel
			three_channels[0].at<char>(y,x) = camera->imageGetBlue(image, width, x, y);
			/// Get Green channel from camera image and store it to second channel	
			three_channels[1].at<char>(y,x) = camera->imageGetGreen(image, width, x, y);
			/// Get Red channel from camera image and store it to third channel	
			three_channels[2].at<char>(y,x) = camera->imageGetRed(image, width, x, y);	
		}
	}
	/// merge the three channels to output Matrix		
	merge(three_channels, out_Mat); 
	/// output: Image converted to BGR memory storage
	return out_Mat;
}

/// Function to set vehicle speed 
void webotsAPI::set_speed(Driver *driver, long double speed)
{
	/// Limit the speed at Max vehicle speed.
	if (speed > TOP_SPEED)
	{
		speed = TOP_SPEED;
	}
	/// update and set vehicle speed to the requested speed
	webotsAPI::speed = speed;
#ifdef DEBUGALL
	printf("\t[DEBUGALL]\t[webotsAPI::set_speed]setting speed to %Lf km/h\n", speed);
#endif
	driver->setCruisingSpeed(speed); /// wbu_driver_set_cruising_speed(kmh);
}

/// Function to set steering angle
void webotsAPI::set_steering_angle(Driver *driver, double wheel_angle)
{
	/// positive: turn right, negative: turn left
	driver->setSteeringAngle(-wheel_angle); /// wbu_driver_set_steering_angle(wheel_angle); 
}

/// Function to compute GPS speed
void webotsAPI::compute_gps_speed(GPS *gps)
{
	double speed_ms = gps->getSpeed();/// wb_gps_get_speed(gps);
	if (isnan(speed_ms)) /// if speed = NaN, set speed to 0.
		speed_ms = 0.0;
	gps_speed = speed_ms * 3.6;  		/// convert speed from m/s (default gps speed) to km/h
#ifdef DEBUGALL
	printf("\t[DEBUGALL]\t[webotsAPI::compute_gps_speed]		GPS speed:  %.1f km/h\n", gps_speed);
#endif
	compute_gps_coords(gps); /// whenever speed is computed, the gps coordinates are updated
}

/// Function to compute GPS coordinates
void webotsAPI::compute_gps_coords(GPS *gps)
{
	const double *coords = gps->getValues(); /// wb_gps_get_values(gps);
#ifdef DEBUGALL
	printf("\t[DEBUGALL]\t[webotsAPI::compute_gps_coords]		Positional vertices are: %f,%f,%f \n",coords[0],coords[1],coords[2]);
#endif
	memcpy(gps_coords, coords, sizeof(gps_coords));
	positionx.push_back(coords[0]);
	positiony.push_back(coords[2]);
}

/// Function to update speedometer display
void webotsAPI::update_display(Display *display, Driver *driver)
{
	const double NEEDLE_LENGTH = 50.0; 			/// speedometer needle length
	display->imagePaste(speedometer_image, -50, 0, false);  /// display background; wb_display_image_paste(display, speedometer_image, 0, 0, false);
	double current_speed = driver->getCurrentSpeed(); 	/// wbu_driver_get_current_speed();
	if (isnan(current_speed)) /// if speed = NaN
		current_speed = 0.0;
	
	/// draw speedometer needle
	double alpha = current_speed / 260.0 * 3.72 - 0.27; 	/// speedometer needle angle
	int x = -NEEDLE_LENGTH * cos(alpha);			/// speedometer needle x-axis projection
	int y = -NEEDLE_LENGTH * sin(alpha);			/// speedometer needle y-axis projection
	display->drawLine(100, 95, 100 + x, 95 + y); /// draw speedometer needle; wb_display_draw_line(display, 100, 95, 100 + x, 95 + y);

	/// draw text
	char txt[64];
	sprintf(txt, "GPS coords: %.1f %.1f", gps_coords[X], gps_coords[Z]);
	display->drawText(txt, 10, 130); //wb_display_draw_text(display, txt, 10, 130);
	sprintf(txt, "GPS speed:  %.1f", gps_speed);
	display->drawText(txt, 10, 140); //wb_display_draw_text(display, txt, 10, 140);
}

/// Function to actuate the steering angle
void webotsAPI::sim_actuate(Driver *driver, long double steering_angle) {
	if (fabs(steering_angle) > 100.0f)
	{
		// transfer the steering angle to global variable.
		webotsAPI::steer90 = steering_angle;
		// Set the steering angle after removing the offset angle
		if (webotsAPI::steer90 > 0)
		{
			// In case of right turn
			driver->setSteeringAngle((webotsAPI::steer90 - webotsAPI::offset_value)); //wbu_driver_set_steering_angle((steer90 - offset_value));
		}
		else
		{
			// In case of left turn
			driver->setSteeringAngle((webotsAPI::steer90 + webotsAPI::offset_value)); //wbu_driver_set_steering_angle(steer90 + offset_value);
		}
	}
	else
	{
		webotsAPI::set_steering_angle(driver, steering_angle);
	}
}

/// Function to read CSV file with 2 columns into a vector of <vector<double>> 
vector<vector<double>> webotsAPI::read_csv(string filename ){
    // Reads a CSV file with 2 columns into a vector of <vector<double>>
    vector<vector<double>> result(2);

    // Create an input filestream
        //const char *c = filename.c_str();
    ifstream myFile(filename.c_str());

    // Make sure the file is open
    if(!myFile.is_open()) throw runtime_error("Could not open file");

    // Helper vars
    string line;
    double val;

        if(myFile.good())
        {
                // Read data, line by line
                while(getline(myFile, line))
                {
                        // Create a stringstream of the current line
                        stringstream ss(line);

                        // Keep track of the current column index
                        int colIdx = 0;
                        // Extract each integer
                        while(ss >> val){

                                // Add the current integer to the 'colIdx' column's values vector
                                result[colIdx].push_back(val);

                                // If the next token is a comma, ignore it and move on
                                if(ss.peek() == ',') ss.ignore();

                                // Increment the column index
                                colIdx++;
                        }
                }
        }

    // Close file
    myFile.close();
    return result;
}

/// Function to getyL from the given (x,y) coordinates of the vehicle and pre-computed reference (GPS coordinates of the centre of lane)
double webotsAPI::getyL(string ref, double positionx , double positiony)
{
        typedef boost::geometry::model::d2::point_xy<double> point_type;
        typedef boost::geometry::model::linestring<point_type> linestring_type;
#ifdef DEBUGALL
        cout << "\t[DEBUGALL]\t[webotsAPI::getyL] Entry: Read CSV" << endl;
#endif
   	vector<vector<double>> gold_ref = read_csv(ref);
#ifdef DEBUGALL
        cout << "\t[DEBUGALL]\t[webotsAPI::getyL] Exit: Read CSV" << endl;
	cout << "\t[DEBUGALL]\t[webotsAPI::getyL] GPS x=" << positionx << "\ty=" << positiony << endl;
#endif

        point_type p(positionx, positiony);
        linestring_type line;
        double x = 0;
        double y = 0;

        for (unsigned int i = 0; i < gold_ref[0].size(); ++i)
    	{
                for(unsigned int j = 0; j < gold_ref.size(); ++j)
                {
                        if (j == 0) x = gold_ref[j][i];
                        else y = gold_ref[j][i];

                }
                line.push_back(point_type(x,y));
#ifdef DEBUGALL
        	//cout << "[webotsAPI::getyL] x= "<< x << " , y = " << y << "\n" ;
#endif
    	}
	/// compute the Point-line
        double yl = boost::geometry::distance(p, line); 
	/// numerical precision --> accuracy to 1 mm
	int yl_x10p3 = (yl*1000);  
	yl = ((float)yl_x10p3)/1000;
#ifdef DEBUGALL
        cout << "\t[DEBUGALL]\t[webotsAPI::getyL] Point-Line: " << yl << endl;
#endif      
	return yl;
}

/// Function to calculate the current actual lateral deviation of the vehicle
long double webotsAPI::calculate_actual_deviation(GPS *gps, int world_encode)
{
	compute_gps_coords(gps);
	long double actual_yL;
	if (world_encode==3) /// city.wbt
		actual_yL = getyL(city_ref_csv, gps_coords[0], gps_coords[2]) - (ROAD_WIDTH/2); //- FINE_TUNE; // As gold_ref is the middle of the entire road, not lane
	else /// city_straight.wbt
		actual_yL = getyL(gold_ref_csv, gps_coords[0], gps_coords[2]) - (ROAD_WIDTH/2);// - FINE_TUNE;
#ifdef DEBUGALL
        cout << "\t[DEBUGALL]\twebotsAPI::calculate_actual_deviation] yL=" << actual_yL << endl;
#endif 	
	/// numerical precision --> accuracy to 1 cm
	int yL_x10p2 = (actual_yL*100);  
	actual_yL = ((float)yL_x10p2)/100;
	return actual_yL;			
}

/// Function to initialise the webots scene
void webotsAPI::initialise(Driver *driver, float update_period)
{
	/// Get the number of devices in the webots scene driver
	for (int device_index = 0; device_index < driver->getNumberOfDevices(); ++device_index) // wb_robot_get_number_of_devices()
	{
		Device *device = driver->getDeviceByIndex(device_index); // wb_robot_get_device_by_index(device_index)
		const string name = device->getName();					 // wb_device_get_name(device)
		/// If there is a speedometer display found, raise a flag that the display is found.		
		if (name == "display")									 //(strcmp(name, "display") == 0)
		{
			enable_display = true;
		}
		/// if any one of the camera is found, raise a flag that the camera has been found.
		else if (name == "camera_front") //(strcmp(name, "camera_front") == 0)
		{
			has_camera = true;
		}
		/// If there is a gps, raise a flag that webots scene has gps.
		else if (name == "gps") //(strcmp(name, "gps") == 0)
		{
			has_gps = true;
		}
	}

	/// enable gps to know position of car. The refresh rate of gps is the 'period'
	if (has_gps)
	{
		gps = driver->getGPS("gps");  // wb_robot_get_device("gps")
		gps->enable(static_cast<int>(update_period)); //wb_gps_enable(gps, TIME_STEP)
	}

	/// Enable camera, if found, with frame rate = period and default width, height and fov in webotsAPI.
	if (has_camera)
	{
		camera_front = driver->getCamera("camera_front"); 	  /// wb_robot_get_device("camera_front");
		camera_front->enable(static_cast<int>(update_period));		  /// wb_camera_enable(camera_front, TIME_STEP);
		camera_width = camera_front->getWidth();		  /// wb_camera_get_width(camera_front);
		camera_height = camera_front->getHeight();		  /// wb_camera_get_height(camera_front);
		camera_fov = camera_front->getFov();			  /// wb_camera_get_fov(camera_front);
	}

	/// Enable display, if found. 
	if (enable_display)
	{
		display = driver->getDisplay("display"); /// wb_robot_get_device("display");
		speedometer_image = display->imageLoad(speedometer_image_path); /// wb_display_image_load(display, "speedometer.png");
	}

	/// If camera is found, start the car by setting speed at VEH_SPEED. 
	/// This speed is the cruising speed the vehicle will achieve respecting the system dynamics and physics.
	if (has_camera)
	{
		set_speed(driver,VEH_SPEED);
	}

	/// Enable features of the car. Hazard flashers, Dipped beams, Anti fog lights and wiper.
	driver->setHazardFlashers(true);	//!< wbu_driver_set_hazard_flashers(true);
	driver->setDippedBeams(true);		//!< wbu_driver_set_dipped_beams(true);
	driver->setAntifogLights(true);		//!< wbu_driver_set_antifog_lights(true);
	driver->setWiperMode(driver->SLOW);  	//!< wbu_driver_set_wiper_mode(SLOW);
}


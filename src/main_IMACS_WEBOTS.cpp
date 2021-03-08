/** @file main_IMACS_WEBOTS.cpp
 *  @brief The MAIN file for IMACS framework using WEBOTS simulator
 */
#include "config_webots.hpp"

#define UNUSED(x) (void)(x) //!< macro for tagging unused images

int main(int argc, char **argv)
{
	using namespace std;
	using namespace cv;
	/// ------- simulation parameters ----------//
	unsigned int scenario = 1; //!< the system scenario to simulate
	unsigned int approximate_pipeline_version = 0; //!< the approximate pipeline version to simulate. See IEEE Access paper. 0: no approximation
	long double yL = 0.0L, actual_yL = 0.0L;   //!< lateral deviation of the LKAS at the look-ahead distance and the actual yL
	float initial_wait_time = 3.5; 	//!< initial simulation wait time to reach required speed
	float sim_limit = 100; 		//!< simulation time limit in seconds
	unsigned int world_encode = 1; 	//!< 1: city_straight.wbt, 2: city_straight_night.wbt
	Mat img_isp; 			//!< Matrix to store ISP image	
	Mat img_webots;			//!< Matrix to store the image captured by webots	
	long double steering_angle =0.0f, prev_steering_angle=0.0f,actuate_steering_angle=0.0f; //!< steering angle from lateral controller
															   //!< previous steering angle. In case of pipelined controllers, we need an array of size equal to the number of pipes (atleast) to store all the previous steering angles
															   //!< the steering angle to actuate at the current step
	long int steering_angle_x10p6=0;	//!< steering angle * 10^6 -> to deal with numerical precision
	/// ----- Reading parameters from the command line-----//
	if (argc < 2)
	{
		cout << "[ERROR] Not enough input arguments\n Usage: ./imacs_webots <unsigned int scenario> [<unsigned int world_encode>] [<unsigned int approximate pipeline version>]\n\t\tworld_encode= 1: city_straight (DEFAULT), 2: city.wbt";
		return -1;
	}
	if (argc >= 4)
	{
		approximate_pipeline_version = atoi(argv[3]);
	} 
	if (argc >= 3)
	{
		world_encode = atoi(argv[2]);
	}

	scenario = atoi(argv[1]);
	cout << "[main webots] scenario: s" << scenario << "\tapproximate pipeline version: v"<< approximate_pipeline_version << endl;	
	
	/// ------- Create class instances and initialise ----------//
	webotsAPI WebotsAPI; 				//!< create an instance of webotsAPI class and call the constructor
	//lkasModel LKASmodel; 				//!< create an instance of lkasModel class and call the constructor
	imageSignalProcessing ISP; 			//!< create an instance of imageSignalProcessing class and call the constructor
	laneDetectionWEBOTS Lane_detection; 		//!< create an instance of laneDetectionWEBOTS class and call the constructor
	lateralControllerWEBOTS Controller(VEH_SPEED); 	//!< create an instance of lateralControllerWEBOTS class and call the constructor
	pathsIMACS paths; 				//!< create an instance of pathsIMACS class and call the constructor

	/// Check if the entered scenario's period, tau_ms (and controller) is defined
	if (scenario >= Controller.period_ms.size())
		throw range_error( "\n[IMACS ERROR] Controller period and delay for the scenario is not defined. \nScenario starts from 0.\nAdd period_ms, tau_ms, m_phi, m_Gamma, m_T and m_K2c for the entered scenario in lateralControllerWEBOTS.hpp");
	
	// ------- simulation main loop -------//
	float period = round(Controller.period_ms[scenario]); //!< Variable to choose the sampling period for simulation
	float delay = round(Controller.tau_ms[scenario]);     //!< Variable to choose the sensor-to-actuator delay for simulation
	cout << "[main webots] period = " << period << " ms,\t delay = " << delay << " ms" << endl;	

	Driver *driver = new Driver(); 		//!< initialise Webots driver
	WebotsAPI.initialise(driver,period);	//!< initialise Webots Simulator (devices, camera frame rate with period and vehicle speed with VEH_SPEED)

	int loop_count_delay = delay/driver->getBasicTimeStep() - 1; 	//!< Keep track of delay during simulation. This is done to get the image at tau + h [first frame is skipped]
	int loop_count_period = period/driver->getBasicTimeStep() - 1;  //!< Keep track of period during simulation.
	int it_counter_period = 0;	//!< counts the kth instance of period
	int it_counter_delay = 0;	//!< counts the kth instance of delay
	bool first_actuate = 0;	 	//!< flag to check whether the first controller actutation is done in the webots simulation
	
	/// --- Initialise: logging results to file ---//
	//std::ofstream outfile1(paths.results+"/results_yL_steering_angle_Case0.csv", std::ios_base::app); /// appending results to a file --> std::ios_base::app
	std::ofstream outfile1(paths.results+"/results_yL.csv", std::ofstream::trunc); /// To erase the contents of the file and re-write --> std::ofstream::trunc
	outfile1 << "simulation time,yL"<< endl;
	std::ofstream outfile_df(paths.results+"/results_steering_angle.csv", std::ofstream::trunc); /// file to store steering angle
	outfile_df << "simulation time,steering angle"<< endl;
	std::ofstream outfile_yL(paths.results+"/results_actual_yL.csv", std::ofstream::trunc); /// file to store actual yL
	outfile_yL << "simulation time,actual_yL"<< endl;

	/// --- Initial time delay loop to reach the required speed ---//
	while (driver->step() != -1 && driver->getTime() < initial_wait_time) //!< using "driver->getCurrentSpeed() < VEH_SPEED" condition does not work as initial few frames are not captured by camera and is black. Hence, simulating for initial_wait_time with steering angle = 0.
	{
		WebotsAPI.sim_actuate(driver,0);  //!< actuate the car with steering angle = 0
		actual_yL=WebotsAPI.calculate_actual_deviation(WebotsAPI.gps,world_encode);
		outfile_yL << to_string(driver->getTime()) + "," + to_string(actual_yL) << endl;
		cout << "[initialising cruising speed] t = " << driver->getTime() << "\tyL=" << actual_yL << "\t Current speed="<< driver->getCurrentSpeed() << endl; //WebotsAPI.gps-> getSpeed()
	}

	/// --- Main loop for processing ---//
	while (driver->step() != -1 && driver->getTime() < sim_limit) //!< wbu_driver_step() --> steps forward in webots simulation && simulate until sim_limit
	{			
		if (loop_count_period == 0) { //!< when simulation time is an integral multiple of sampling period, i.e. simulation time = n*(sampling period)
			const unsigned char *camera_front_image = NULL; //!< to capture the camera image from WEBOTS simulator camera

			/// If cameras are found, do the main processing.
			if (WebotsAPI.has_camera)
			{
				/// --- Image capture from Webots -----------//				
				camera_front_image = WebotsAPI.camera_front->getImage(); //wb_camera_get_image(camera_front);
				UNUSED(camera_front_image);
				img_webots = WebotsAPI.webots_img_2_Mat(WebotsAPI.camera_front, camera_front_image);				
				cout << "[main_webots] Capture Image: t = " << driver->getTime() << " sec, Img = " << it_counter_period << ", actual deviation yL = " << actual_yL <<endl;

				/// ------- imageSignalProcessing ----------//
				img_isp = ISP.approximate_pipeline(img_webots, approximate_pipeline_version);
				
				/// ------- laneDetection -----------------// 
				yL = Lane_detection.lane_detection_pipeline(img_isp, world_encode);
				cout << "[main_webots] \tSensing & Processing Task: Perform Lane Detection, yL at look-ahead distance =" << yL << endl;
				/// --- store to file----//
				outfile1 << to_string(driver->getTime()) + "," + to_string(yL) << endl;
				
				/// ------- control_compute --------------//
				prev_steering_angle = steering_angle;// to make the actuation part use the old steering angle, only used for tau == period
				Controller.compute_steering_angle(yL,it_counter_period,scenario);
				steering_angle = Controller.get_steering_angle();
				Controller.estimate_next_state(it_counter_period,scenario);
				cout << "[main_webots] \t\tControl Computation Task : steering angle computed for Img " << it_counter_period << " is df = " << steering_angle << endl;
			}
			it_counter_period++;
			loop_count_period = period/driver->getBasicTimeStep() - 1;
		}
		else {
			loop_count_period--;
		}
		if (loop_count_delay == 0) { //!< when simulation time = sensor-to-actuator delay + n*(sampling period)

			if (WebotsAPI.has_camera && it_counter_delay > 0)
			{
				/// ------- Actuate -------------------//
				/// (kind of) dealing with numerical precision issue with steering angle computed by the controller
				steering_angle_x10p6 = prev_steering_angle*1000000;
				actuate_steering_angle = ((float)steering_angle_x10p6)/1000000;
				WebotsAPI.sim_actuate(driver,actuate_steering_angle);					
				cout << "[main_webots] \t\t\tActuation Task: actuate at t = " << driver->getTime() << ", steering angle = "<< actuate_steering_angle << "sec for Img = " << it_counter_delay-1 << endl;
				first_actuate = 1;
				outfile_df << to_string(driver->getTime()) + "," + to_string(actuate_steering_angle) << endl;
			}
				
			it_counter_delay++;
			loop_count_delay = period/driver->getBasicTimeStep() - 1;
		}
		else {
			loop_count_delay--;
		}
		/// ---- Compute Actual yL ----//
		actual_yL=WebotsAPI.calculate_actual_deviation(WebotsAPI.gps,world_encode);
		outfile_yL << to_string(driver->getTime()) + "," + to_string(actual_yL) << endl;
	}
	cout << "[main_webots] SUCCESS: Simulation Completed!!" << endl;
	delete driver;

	return 0;
}




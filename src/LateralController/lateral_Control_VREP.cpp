/** @file lateral_Control_VREP.cpp
 *  @brief The source file for lateral controller implementation when using IMACS framework with VREP simulator
 */
#include "lateral_Control_VREP.hpp"

using namespace std;
using namespace Eigen;

// class methods
void lateralControllerVREP::compute_steering_angle(long double the_yL, int the_it_counter, int scenario) {
    m_z3 = the_yL; /// update the state measurement yL
    m_z5 = 2 * m_z3 / ( pow( m_LL + m_l_r, 2 ) );   /// curvature calculate

    Matrix<long double, LENGTH_PHI_AUG, 1> zt_temp, zt;       /// zt is the transferred state vector
    if (the_it_counter == 0){
        zt_temp <<  m_z1,
                    m_z2,
                    m_z3,
                    m_z4,
                    m_z5,
                    0.0L;
    } else {
        zt_temp <<  m_z1,
                    m_z2,
                    m_z3,
                    m_z4,
                    m_z5,
                    m_input[the_it_counter-1];
    }

    zt = m_T[scenario] * zt_temp;        

    Matrix<long double, (LENGTH_PHI_AUG-1), 1> zt_temp_2;
    zt_temp_2 << zt[1], 
                 zt[2], 
                 zt[3], 
                 zt[4], 
                 zt[5];

    /// calculate the desired steering angle 
    m_desired_steering_angle = m_K2c[scenario] * zt_temp_2;
    cout << " \tsteering angle: "<< m_desired_steering_angle;	          
}

long double lateralControllerVREP::get_steering_angle() { 
    return m_desired_steering_angle;       
}

void lateralControllerVREP::estimate_next_state(int the_it_counter, int scenario) {
    /// transfer state vector
    Matrix<long double, LENGTH_PHI_AUG, 1> zkp_temp, zkp;  
    if (the_it_counter == 0){
        zkp_temp << m_z1,
                    m_z2,
                    m_z3,
                    m_z4,
                    m_z5,
                    0.0L;       
    } else {
        zkp_temp << m_z1,
                    m_z2,
                    m_z3,
                    m_z4,
                    m_z5,
                    m_input[the_it_counter-1]; 
    }

    /// given the control design, estimate next states
    zkp = m_phi_aug[scenario]   * zkp_temp + 
          m_Gamma_aug[scenario] * m_desired_steering_angle;  

        m_z1 = zkp[0];
        m_z2 = zkp[1];
        m_z4 = zkp[3];
        m_z5 = zkp[4];
        m_input[the_it_counter+1] = m_desired_steering_angle;
}


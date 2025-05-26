//==============================================================
// Filename :       ode_withCargo.cpp
// Authors :        Mrinal Magar (s2689529)
//                  Nick Hogenkamp (s2653753)
// Version :        v1.0
// License :
// Description :    Extended ODE dynamics of drone with cargo
//==============================================================


// Include files
#include "../include/dronesim/Ode.h"
#include "../include/dronesim/Ode_withCargo.h"

#include<cmath>
#include<vector>
#include<iostream>


// Function vector_resize
std::vector<float> Ode_withCargo::vector_resize(){
    m_states.push_back(c_states[0]);
    m_states.push_back(c_states[1]);
    m_states.push_back(c_states[2]);
    m_states.push_back(c_states[3]);
    return m_states;
}


// Function changefxu_cargo
std::vector<float> Ode_withCargo::changefxu_cargo(const std::vector<float> &states){

    m_fxu.resize(9);

    float f_rope;
    float f__rope;
    float f_rope_x;
    float f_rope_y;

    m_outputs[0] = sqrt((states[0]-states[5])*(states[0]-states[5])+(states[1]-states[6])*(states[1]-states[6]));
    m_outputs[1] = ((states[0]-states[5])*(states[3]-states[7])+(states[1]-states[6])*(states[4]-states[8]))/m_outputs[0];

    f__rope = k_rope*(m_outputs[0]-l_rope)+d_rope*m_outputs[1];

    if(f__rope > 0){
        f_rope = f__rope;
    }
    else{
        f_rope = 0;
    }
    f_rope_x = f_rope*(states[0]-states[5])/m_outputs[0];
    f_rope_y = f_rope*(states[1]-states[6])/m_outputs[0];

    m_fxu[0] = states[3];
	m_fxu[1] = states[4];
	
	if((states[2] <= -0.7854)&&(m_inputs[1]<0)){ //45 degrees
		m_fxu[2] = 0;
	}
	else if((states[2] >= 0.7854)&&(m_inputs[1] > 0)){
		m_fxu[2] = 0;
	}
	else{
		m_fxu[2] = m_inputs[1];
	}
	
	m_fxu[3] = -(1/drone_mass)*(m_inputs[0]*sin(states[2]) + C_drag*sqrt(states[3]*states[3] + states[4] * states[4])*states[3] + f_rope_x);
	m_fxu[4] = (1 / drone_mass) * (m_inputs[0] * sin(states[2]) - C_drag * sqrt(states[3] * states[3] + states[4] * states[4]) * states[3] - f_rope_y) - g;
    m_fxu[5] = states[7];
    m_fxu[6] = states[8];
    m_fxu[7] = (1/cargo_mass)*( - C_drag*sqrt(states[7]*states[8] + states[8] * states[8])*states[7] + f_rope_x);
    m_fxu[8] = (1/cargo_mass)*( - C_drag*sqrt(states[7]*states[8] + states[8] * states[8])*states[8] + f_rope_y) - g;

    return m_fxu;
}
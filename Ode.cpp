//==============================================================
// Filename :       Ode.cpp
// Authors :        Mrinal Magar (s2689529)
//                  Nick Hogenkamp (s2653753)
// Version :        v1.0
// License :
// Description :    Calculating the fxu of the states wrt inputs
//==============================================================


// Include files
#include "../include/dronesim/Ode.h"
#include <cmath>


// Function changefxu
std::vector<float> Ode::changefxu() {
	m_fxu[0] = m_states[3];
	m_fxu[1] = m_states[4];
	
	if((m_states[2] <= -0.7854)&&(m_inputs[1]<0)){ //45 degrees
		m_fxu[2] = 0;
	}
	else if((m_states[2] >= 0.7854)&&(m_inputs[1] > 0)){
		m_fxu[2] = 0;
	}
	else{
		m_fxu[2] = m_inputs[1];
	}
	
	m_fxu[3] = -(1/drone_mass)*(m_inputs[0]*sin(m_states[2]) + C_drag*sqrt(m_states[3]*m_states[3] + m_states[4] * m_states[4])*m_states[3]);
	m_fxu[4] = (1 / drone_mass) * (m_inputs[0] * sin(m_states[2]) - C_drag * sqrt(m_states[3] * m_states[3] + m_states[4] * m_states[4]) * m_states[3]) - g;
	return m_fxu;
}


// Getter m_states
std::vector<float> Ode::getStates() {
	return m_states;
}


// Getter m_inputs
std::vector<float> Ode::getInputs() {
	return m_inputs;
}


// Function updateStates
void Ode::updateStates(const std::vector<float>& STATES) {
	m_states = STATES;
}


// Function updateInput
void Ode::updateInput(const std::vector<float>& INPUTS) {
	m_inputs = INPUTS;
}

//==============================================================
// Filename :       Nim.cpp
// Authors :        Mrinal Magar (s2689529)
//                  Nick Hogenkamp (s2653753)
// Version :        v1.0
// License :
// Description :  Functions of different NIM methods
//==============================================================


// Include files
#include "../include/dronesim/Nim.h"
#include <iostream>


// Function forward_euler
std::vector<float> Nim::forward_euler(const std::vector<float> &fxu, const std::vector<float> &states )
{
	next_itr.resize(fxu.size());
	
		for (int i = 0; i < fxu.size(); i++)
		{
			next_itr[i] = states[i] + h * fxu[i];
		}
		
	return next_itr;
}


// Function runge_kutta
std::vector<float> Nim::runge_kutta(const std::vector<float>& fxu, const std::vector<float>& states) 
{	
	std::vector<float> temp_states = states;
	std::vector<float> temp_fxu = fxu;

	std::vector<float> K_1 = temp_fxu;
		for(int i = 0; i< temp_states.size(); i++){
			temp_states[i] = temp_states[i] + K_1[i]*h/2;
		}

	temp_fxu = ode_withCargo.changefxu_cargo(temp_states);

	std::vector<float> K_2 = temp_fxu;
		for(int i = 0; i< temp_states.size(); i++){
			temp_states[i] = temp_states[i] + K_2[i]*h/2;
		}
	temp_fxu = ode_withCargo.changefxu_cargo(temp_states);

	std::vector<float> K_3 = temp_fxu;
		for(int i = 0; i< temp_states.size(); i++){
			temp_states[i] = temp_states[i] + K_3[i]*h;
		}
	temp_fxu = ode_withCargo.changefxu_cargo(temp_states);
	std::vector<float> K_4 = temp_fxu;
	next_itr.resize(temp_fxu.size());
		for (int i = 0; i < temp_fxu.size(); i++)
		{
			next_itr[i] = states[i] + (0.16666) * h * (K_1[i] + 2 * K_2[i] + 2 * K_3[i] + K_4[i]);
		}

	return next_itr;
}


// Function forward_euler_cargo
std::vector<float> Nim::forward_euler_cargo(const std::vector<float> &fxu, const std::vector<float> &states )
{
	next_itr.resize(fxu.size());
	for (int i = 0; i < fxu.size(); i++)
	{
		next_itr[i] = states[i] + h_euler * fxu[i];
	}
	return next_itr;
}


// Function updateInput
void Nim::updateInput(const std::vector<float>& INPUT) {
	ode_withCargo.updateInput(INPUT);
}
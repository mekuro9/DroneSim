//==============================================================
// Filename :       Nim.h
// Authors :        Mrinal Magar (s2689529)
//                  Nick Hogenkamp (s2653753)
// Version :        v1.0
// License :
// Description :    Class nim
//==============================================================


#ifndef NIM_H
#define NIM_H


// Include files
#include "Ode.h"
#include "Ode_withCargo.h"
#include <vector>


// Global variables
const float h = 0.01f; //time step
const float h_euler = 0.0005f;


// Nim (base class)
class Nim {
public:
	// Constructor 
	Nim() {
	}
	
	// Destructor
	~Nim() = default;

	// Passes on the input change to the nim
	void updateInput(const std::vector<float>& INPUT);
	
	// Function to calculate the x+1 from forward_euler method
	std::vector<float> forward_euler(const std::vector<float> &fxu, const std::vector<float> &states);
	
	// Calculate using runge_kutta method
	std::vector<float> runge_kutta(const std::vector<float>& fxu, const std::vector<float>& states);
	
	// Calculate ode with cargo using forward euler
	std::vector<float> forward_euler_cargo(const std::vector<float> &fxu, const std::vector<float> &states);
	
private:
	std::vector<float> next_itr;
	Ode_withCargo ode_withCargo;
};

#endif //NIM_H

//==============================================================
// Filename :       Ode.h
// Authors :        Mrinal Magar (s2689529)
//                  Nick Hogenkamp (s2653753)
// Version :        v1.0
// License :
// Description :    class ode
//==============================================================


#ifndef ODE_H
#define ODE_H


// Include files
#include <vector>


// Global variables
const float drone_mass = 3.0f;	    // mass
const float C_drag = 0.1f;	// drag constant
const float g = 9.81f;		// grav. acc.


// Class Ode (base class)
class Ode {
public: 
	// Constructor
	Ode() { 
		m_states.resize(5);
		m_states = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
		m_inputs.resize(2);
		m_inputs = { 0.0f, 0.0f };
		m_fxu.resize(5);
	}

	// Destructor
	~Ode() = default;
	
	std::vector<float> changefxu();
	std::vector<float> getStates();
	std::vector<float> getInputs();

	// Function update states
	void updateStates(const std::vector<float>& STATES);
	
	// Function update input
	void updateInput(const std::vector<float>& INPUTS);

protected:
	// states
	std::vector<float> m_states; 
	std::vector<float> m_inputs;
	std::vector<float> m_fxu;
};


#endif

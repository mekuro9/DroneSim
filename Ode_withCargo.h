//==============================================================
// Filename :       ode_withCargo.h
// Authors :        Mrinal Magar (s2689529)
//                  Nick Hogenkamp (s2653753)
// Version :        v1.0
// License :
// Description :    child class of ode
//==============================================================
#ifndef Ode_withCargo_H
#define Ode_withCargo_H

#include "Ode.h"
#include <vector>

//cargo parameters
const float cargo_mass = 2;
const float C_drag_cargo = 0.1;
const float l_rope = 1.5;
const float k_rope = 40000;
const float d_rope = 50;


class Ode_withCargo : public Ode{
    
public:

    Ode_withCargo(){ //constructor
        m_inputs.resize(2);
        m_states.resize(5);
        m_outputs.resize(2);
        m_outputs = { 1.5f, 0.0f };
        c_states.resize(4);
        c_states = {1.0f, 1.0f, 0.0f, 0.0f};
    }
    ~Ode_withCargo()=default;

    std::vector<float> vector_resize();

    std::vector<float> changefxu_cargo(const std::vector<float> &m_states);
protected:
     
     std::vector<float> m_outputs;
     std::vector<float> c_states;

};

#endif

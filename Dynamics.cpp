//==============================================================
// Filename :       Dynamics.cpp
// Authors :        Mrinal Magar (s2689529)
//                  Nick Hogenkamp (s2653753)
// Version :        v1.0
// License :  
// Description :    Dynamics node which uses NIM and ODE 
//				    to compute next states
//==============================================================


// Include files
#include <chrono>
#include <memory>
#include <iostream>

#include "../include/dronesim/Nim.h"
#include "../include/dronesim/Ode.h"
#include "../include/dronesim/Ode_withCargo.h"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;


// Global variables
const std::vector<float> initInput = {0, 0};
const std::vector<float> initState = {1, 1, 0, 0, 0};


// Derived class Dynamics from base class Node
class Dynamics : public rclcpp::Node {
public:
	// Constructor	
	Dynamics() : Node("Dynamics"), count_(0){
		std::cout << "init...\n"; 
		this->init();
		input_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("inputs",10, std::bind(&Dynamics::topic_callback, this, _1));
		nim_subscriber_ = this->create_subscription<std_msgs::msg::Bool>("method", 10, std::bind(&Dynamics::method_callback, this, _1));
		publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("states", 10);    // CHANGE
		timer_ = this->create_wall_timer(250ms, std::bind(&Dynamics::timer_callback, this));
	  }


private:
	// Function next states
	std::vector<float> nextStates(const std::vector<float>& resultOfNim) {
		nextState = resultOfNim;
		return nextState;
	}


	// Function topic_callback
	void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
		this->Inputs = msg->data;
		ode_withoutCargo.updateInput(this->Inputs);
		odeWithCargo.updateInput(this->Inputs);
		nim_method.updateInput(this->Inputs);
		RCLCPP_INFO(this->get_logger(), "Inputs: Thrust '%f'  Angular vel '%f' ", msg->data[0], msg->data[1]);
	}


	// Function method_callback
	void method_callback(const std_msgs::msg::Bool::SharedPtr msg) {
		// False for forward euler and true for runge_kutta
		method = msg->data;
	}


	// Function input 
	// Input a 0 for a drone without a cargo
	// Input a 1 for a drone with a cargo
	bool init_ip() {
		bool input;
		std::cout<<"Input 1 for drone with cargo and 0 for drone without cargo: ";
		std::cin>>input;
		return input;
	}


	// Initialization
	void init() {
		// Init inputs/states & Allocate space
		this->Inputs = { initInput[0], initInput[1] };
		this->States = { initState[0], initState[1], initState[2], initState[3], initState[4]};
			if(init_ip() == true) { 
				std::cout << "Cargo drone selected...\n";
				this->States = odeWithCargo.vector_resize();
				odeWithCargo.updateInput(this->Inputs);
				odeWithCargo.updateStates(this->States);
				cargo_attached = true;
			}
			else {
				std::cout << "Drone without cargo selected...\n";
				ode_withoutCargo.updateInput(this->Inputs);
				ode_withoutCargo.updateStates(this->States);
				cargo_attached = false;
			}
		this->fxu.resize(States.size());
		this->nextItr.resize(fxu.size());
	}


	void timer_callback() {
		// NIM step
		if((cargo_attached==true)&&(method == true)){
			std::cout << "Cargo RK\n";
			this->fxu = odeWithCargo.changefxu_cargo(States);
			this->nextItr = nextStates(nim_method.runge_kutta(fxu, States));
			odeWithCargo.updateStates(nextItr);
			this->States = odeWithCargo.getStates();
		}
		else if((cargo_attached==true)&&(method == false)){
			std::cout << "Cargo Euler\n";
			this->fxu = odeWithCargo.changefxu_cargo(States);
			this->nextItr = nextStates(nim_method.forward_euler_cargo(fxu, States));
			odeWithCargo.updateStates(nextItr);
			this->States = odeWithCargo.getStates();
		}
		else{
			std::cout << "noCargo Euler\n";
			this->fxu = ode_withoutCargo.changefxu();
			this->nextItr = nextStates(nim_method.forward_euler(fxu, States));
			ode_withoutCargo.updateStates(nextItr);
			this->States = ode_withoutCargo.getStates();
		}

		// Prep message 
		auto message = std_msgs::msg::Float32MultiArray();  
	
			for(int i = 0; i < (int)States.size(); i++){
				message.data.push_back(States[i]);
				
				RCLCPP_INFO(this->get_logger(), "Publishing Updated States: '%d', '%f'", i, message.data[i]);
			}
		
		publisher_->publish(message);
	} 

	// ODE & NIM
	std::vector<float> Inputs;
	std::vector<float> States;
	std::vector<float> fxu;
	std::vector<float> nextItr;
	std::vector<float> nextState;
	bool cargo_attached;
	bool method = true;

	Ode ode_withoutCargo;
	Nim nim_method;
	Ode_withCargo odeWithCargo;


	// Publisher and Subscription
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;  
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr input_subscriber_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nim_subscriber_ ;
	size_t count_;
};


// Main function
// Code starts here
int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Dynamics>());
	rclcpp::shutdown();
	return 0;
}

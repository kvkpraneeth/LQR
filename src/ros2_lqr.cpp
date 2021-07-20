#include "lqr/lqr.h"
#include "rclcpp/rclcpp.hpp"
#include "lqr/ros2_lqr.h"

lqr_node::lqr_node() : Node("Linear Quadratic Regulator")
{

    this->declare_parameter<int>("number_of_lqr_loops", 1);
    this->get_parameter<int>("number_of_lqr_loops", this->number_of_lqr_loops_);

    this->A_.resize(this->number_of_lqr_loops_);
    this->B_.resize(this->number_of_lqr_loops_);
    this->Q_.resize(this->number_of_lqr_loops_);
    this->R_.resize(this->number_of_lqr_loops_);

    this->declare_parameter<std::vector<std::string>>("system_names", std::vector<std::string>());
    this->get_parameter<std::vector<std::string>>("system_names", this->SN_);

    int j = 0;

    for(auto& name : SN_)
    {
        this->declare_parameter<std::vector<double>>(name+".A", std::vector<double>());        
        this->get_parameter<std::vector<double>>(name+".A", this->A_[j]);
        
        this->declare_parameter<std::vector<double>>(name+".B", std::vector<double>());        
        this->get_parameter<std::vector<double>>(name+".B", this->B_[j]);
        
        this->declare_parameter<std::vector<double>>(name+".Q", std::vector<double>());        
        this->get_parameter<std::vector<double>>(name+".Q", this->Q_[j]);
        
        this->declare_parameter<std::vector<double>>(name+".R", std::vector<double>());        
        this->get_parameter<std::vector<double>>(name+".R", this->R_[j]);
        
        j = j+1;
    }
    
}

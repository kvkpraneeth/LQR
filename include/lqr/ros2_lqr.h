#ifndef ROS2_LQR_H
#define ROS2_LQR_H

#include "rclcpp/rclcpp.hpp"
#include "lqr/lqr.h"

class lqr_node : public rclcpp::Node
{
    
    public:
        
        //Defaults.
        lqr_node();

    private:

        /*
        This parameter is for deciding the number_of_lqr_loops_ required.
        When would such a situation arise?
        If designing for a quadrotor, this node would act as a control block
        than one of many controllers for each rotor.
        */
        
        int number_of_lqr_loops_; 

        // When multiple loops come, multiple A, B, Q, R matrices come.

        std::vector<std::vector<double>> A_;
        std::vector<std::vector<double>> B_;
        std::vector<std::vector<double>> Q_;
        std::vector<std::vector<double>> R_;

        // System Names.
        std::vector<std::string> SN_;

};



#endif

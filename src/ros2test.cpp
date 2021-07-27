#include "lqr/lqr.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <functional>
#include <iostream>
#include <rclcpp/executor.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/detail/float32__struct.hpp>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>
#include <string>
#include <vector>

class testNode : public rclcpp::Node
{
    public:

        // Constructor
        testNode() : Node("testNode")
        {   
        
            joints.resize(6);

            for(int i=0; i<6; i++)
            {
                joints[i] = this->create_publisher<std_msgs::msg::Float32>("/joint"+std::to_string(i+1)+"_control", 100);
            }

            jointStateSub = this->create_subscription<std_msgs::msg::Float32MultiArray>("abbState", 100,std::bind(&testNode::callback, this,std::placeholders::_1)); 

            jointDesiredSub = this->create_subscription<std_msgs::msg::Float32MultiArray>("desState", 100, std::bind(&testNode::desStateCallback, this, std::placeholders::_1));

            System.A = Eigen::MatrixXd::Identity(6,6);
            System.B = Eigen::MatrixXd::Identity(6,6);

            Q.resize(36);
            R.resize(36);

            for(int i=0; i<6; i++)
            {
                for(int j=0; j<6; j++)
                {
                    Q[6*i + j] = 10*(i==j);
                    R[6*i + j] = 1*(i==j);
                }
            }

            lqr l(System, Q, R);
           
            msg_ = std::make_shared<std_msgs::msg::Float32MultiArray>();

            msg_->data.resize(6);

            for(int i = 0; i < 6; i++)
            {
                msg_->data[i] = 1.57;
            }

        }


        //Publisher Vector
        std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> joints;

        //Get Joint States
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr jointStateSub;

        //Joint Desired States
        std_msgs::msg::Float32MultiArray::SharedPtr msg_;

        //Desired Joint States Subscriber
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr jointDesiredSub;
            
        //Joint States Callback
        void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {           
            
            Eigen::MatrixXd E = Eigen::MatrixXd(msg->data.size(),1);
            
            Eigen::MatrixXd U = Eigen::MatrixXd(joints.size(), 1);

            while(msg_->data.empty())
            {
                int i = 0;
                (void) i;
            }

            for(unsigned long x = 0; x<msg->data.size(); x++)
            {
                E(x,0) = msg_->data[x] - msg->data[x];
            }

            U = System.K * E;
            
            for(int x = 0; x < U.size(); x++)
            {
                std_msgs::msg::Float32 uk;
                uk.data = U(x,0);
                joints[x]->publish(uk);
            }

        }

        void desStateCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){msg_=msg;};

        //System Description.
        LinearStateSpace System;

        //Error and Effort Values.
        std::vector<double> Q, R; 

};

int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exe;

    std::shared_ptr<testNode> tn = std::make_shared<testNode>();
    exe.add_node(tn->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}

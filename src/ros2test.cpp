#include "stdlib.h"
#include "rclcpp/rclcpp.hpp"

class testNode : public rclcpp::Node
{
    public:

        testNode() : Node("testNode")
        {

                        

        }

        double f=10.0;
        std::chrono::duration <double, std::ratio<1,1000>> frequency{f};
        std::shared_ptr<rclcpp::TimerBase> timer = 
            this->create_wall_timer(frequency, std::bind(&testNode::test, this));
        
        std::vector<std::string> joint_names;
        
        void test();


};
int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);


    rclcpp::shutdown();

    return 0;
}

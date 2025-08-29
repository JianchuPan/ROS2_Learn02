#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include "cppNode.h"

void CPPNode() {
    std::cout << "A CPP Node!" << std::endl;
}

int main(int argc, char * argv[]) {
    CPPNode();
    std::cout << "参数数量:"<<argc<< std::endl;
    std::cout << "程序名字:"<<argv[0]<< std::endl;
    if(argc < 2) {
        std::cout << "没有参数"<< std::endl;
    }
    else{
        std::string arg1 = argv[1];
        if(arg1 == "--help" || arg1 == "-h") {
            std::cout << "程序帮助:"<<"这是一个用来学习ROS2 Node节点的程序！"<< std::endl;
        }
    }

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("hello_world_cppnode");
    RCLCPP_INFO(node->get_logger(), "Hello, ROS2 World from C++ Node!");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;   
}
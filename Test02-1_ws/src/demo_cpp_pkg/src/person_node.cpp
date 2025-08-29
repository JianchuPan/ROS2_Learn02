#include "rclcpp/rclcpp.hpp"

class PersonNode :public rclcpp::Node
{
private:
    std::string name_;
    int age_;
public:
    PersonNode(const std::string &node_name,const std::string &name,const int &age)
    :Node(node_name) // 调用父类构造函数
    {
        this -> name_ = name;
        this -> age_ = age;
    }
    
    void eat(const std::string &foodName) // const的作用是保证函数内不修改类成员变量
    {
        RCLCPP_INFO(this->get_logger(), "我是%s,%d岁,I like eating %s", this->name_.c_str(),
        this->age_,foodName.c_str());
    };
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PersonNode>("person_node","Alice", 30); //auto是类型自动推导 
    node->eat("菜菜");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


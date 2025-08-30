#include <QApplication>
#include <QLabel>
#include <QString>
#include "rclcpp/rclcpp.hpp"
#include "status_interfaces/msg/system_status.hpp"

using SystemStatus = status_interfaces::msg::SystemStatus;

class SysStatusDisplay : public rclcpp::Node
{
private:
    rclcpp::Subscription<SystemStatus>::SharedPtr subscription_;
    QLabel* label_;

public:
    SysStatusDisplay(): Node("sys_status_display")
    {
        subscription_ = this->create_subscription<SystemStatus>(
            "sys_status", 10,
            // std::bind(&SysStatusDisplay::status_callback, this, std::placeholders::_1));
            [&](const SystemStatus::SharedPtr msg)->void {
                label_->setText(get_qstr_from_msg(msg));
            });
        label_ = new QLabel(get_qstr_from_msg(std::make_shared<SystemStatus>()));
        label_->resize(500, 300);  // 宽=200像素，高=100像素
        label_->show();
    }

    QString get_qstr_from_msg(const SystemStatus::SharedPtr msg)
    {
        //根据msg内容生成显示字符串
        std::stringstream show_str;
        show_str 
            << "===============系统状态可视化工具=============\n"
            << "数 据 时 间：\t" <<msg->stamp.sec <<"\ts\n"
            << "用  户 名:\t" <<msg->host_name <<"\ts\n"
            << "CPU使用率:\t" <<msg-> cpu_percent<<"\t%\n"
            << "内存使用率：\t" <<msg-> memory_percent<<"\t%\n"
            << "内存总大小：\t" <<msg-> memory_total<<"\tMB\n"
            << "剩余有效内存：\t" <<msg-> memory_available<<"\tMB\n"
            << "网络发送量：\t" <<msg-> net_sent<<"\tMB\n"
            << "网络接收量：\t" <<msg-> net_recv<<"\tMB\n"
            << "=========================================\n";
        return QString::fromStdString(show_str.str());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    auto node = std::make_shared<SysStatusDisplay>();
    std::thread spin_thread([&]() ->void {rclcpp::spin(node);});
    spin_thread.detach();

    app.exec();
    rclcpp::shutdown();

    return 0;
}
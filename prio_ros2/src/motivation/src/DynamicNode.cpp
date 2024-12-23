#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rcutils/cmdline_parser.h"
#include <chrono>
#include <string>
#include <sys/timex.h>

class DynamicNode : public rclcpp::Node
{
public:
    DynamicNode(const std::string & node_name, const std::string & pub_topic_name, const std::string & sub_topic_name)
    : Node(node_name)
    {
        // QoS 설정은 talker와 동일하게 KeepLast(10)으로 설정
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        publisher_ = this->create_publisher<std_msgs::msg::String>(pub_topic_name, qos);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            sub_topic_name, qos, std::bind(&DynamicNode::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        struct ntptimeval t1, t2;
        ntp_gettime(&t1); // 수신 시간 기록

        RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
        printf("%s start %ld.%09ld\n", this->get_name(), t1.time.tv_sec, t1.time.tv_usec);
        
        // 메시지를 다시 발행
        auto new_msg = std::make_unique<std_msgs::msg::String>();
        new_msg->data = msg->data + " (forwarded)";
        
        publisher_->publish(std::move(new_msg));

        ntp_gettime(&t2); // 발행 시간 기록
        printf("%s end %ld.%09ld\n", this->get_name(), t2.time.tv_sec, t2.time.tv_usec);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);  // stdout flush를 위한 설정
    rclcpp::init(argc, argv);

    //if (argc != 4) {
      //  RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Usage: ros2 run <package_name> <executable_name> <node_name> <sub_topic_name> <pub_topic_name>");
       // return 1;
    //}

    std::string node_name = argv[1];
    std::string sub_topic_name = argv[2];
    std::string pub_topic_name = argv[3];

    // DynamicNode 생성
    auto node = std::make_shared<DynamicNode>(node_name, pub_topic_name, sub_topic_name);
    
    // spin을 통해 메시지 처리
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


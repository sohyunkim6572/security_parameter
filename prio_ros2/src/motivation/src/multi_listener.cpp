#include <cstdio>
#include <memory>
#include <string>
#include <unistd.h>
#include <sys/syscall.h>
#include <time.h>
#include <sys/timex.h>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/string.hpp"
#include <sys/types.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <sched.h>

void print_usage() {
    printf("Usage for listener app:\n");
    printf("listener [-t topic_name] [-h]\n");
    printf("options:\n");
    printf("-h : Print this help function.\n");
    printf("-t topic_name : Specify the topic on which to subscribe. Defaults to chatter.\n");
}

class Listener : public rclcpp::Node {
public:
    // 고유한 노드 이름을 받을 수 있도록 생성자 수정
    Listener(const std::string& node_name, const std::string& topic_name, std::chrono::milliseconds period)
    : Node(node_name), node_name_(node_name) {  // 노드 이름을 node_name으로 설정
        auto callback = [this, topic_name](const std_msgs::msg::String::SharedPtr msg) -> void {
            // listener1만 시간 정보를 출력
            if (node_name_ == "listener1") {
                struct ntptimeval t, t2;
                syscall(456, 1, msg->data.c_str());
                ntp_gettime(&t);
                printf("%s start %s %ld.%09ld\n", topic_name.c_str(), msg->data.c_str(), t.time.tv_sec, t.time.tv_usec);

                syscall(456, 2, msg->data.c_str());
                ntp_gettime(&t2);
                printf("%s end %s %ld.%09ld\n", topic_name.c_str(), msg->data.c_str(), t2.time.tv_sec, t2.time.tv_usec);
            } //else {
                // listener1 이외의 노드는 메시지 출력만 수행
                //RCLCPP_INFO(this->get_logger(), "%s heard: '%s'", node_name_.c_str(), msg->data.c_str());
            //}
        };
        sub_ = create_subscription<std_msgs::msg::String>(topic_name, 10, callback);
    }

private:
    std::string node_name_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char* argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
        print_usage();
        return 0;
    }

    rclcpp::init(argc, argv);

    std::string topic = "chatter"; // Default topic
    char* cli_option = rcutils_cli_get_option(argv, argv + argc, "-t");
    if (nullptr != cli_option) {
        topic = std::string(cli_option);
    }

    std::chrono::milliseconds timer_period = std::chrono::milliseconds(1000);
    char* cli_option2 = rcutils_cli_get_option(argv, argv + argc, "-p");
    if (nullptr != cli_option2) {
        timer_period = std::chrono::milliseconds(std::stoi(cli_option2));
    }

    // 고유한 노드 이름을 가진 여러 구독자 노드 생성
    auto listener1 = std::make_shared<Listener>("listener1", topic, timer_period);
    auto listener2 = std::make_shared<Listener>("listener2", topic, timer_period);
    auto listener3 = std::make_shared<Listener>("listener3", topic, timer_period);
    //auto listener4 = std::make_shared<Listener>("listener4", topic, timer_period);
    
    // SingleThreadedExecutor를 사용하여 여러 구독자 노드를 처리
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(listener1);
    executor.add_node(listener2);
    executor.add_node(listener3);
    //executor.add_node(listener4);

    // Executor로 모든 노드 스핀 처리
    executor.spin();

    rclcpp::shutdown();
    return 0;
}


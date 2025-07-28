#include <boost/asio.hpp>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <thread>

using boost::asio::ip::udp;
using namespace std::chrono_literals;

class UdpToCmdVel : public rclcpp::Node {
private:
    void start_receive() {
        socket_.async_receive_from(
            boost::asio::buffer(recv_buffer_), remote_endpoint_,
            [this](boost::system::error_code ec, std::size_t bytes_recvd) {
                if (!ec && bytes_recvd > 0) {
                    std::string data(recv_buffer_.data(), bytes_recvd);
                    float lin=0.0f, ang=0.0f;
                    if (std::sscanf(data.c_str(), "%f %f", &lin, &ang) == 2) {
                        auto msg = geometry_msgs::msg::Twist();
                        msg.linear.x = lin;
                        msg.angular.z = ang;
                        pub_->publish(msg);
                        RCLCPP_DEBUG(this->get_logger(),
                                     "Published Twist(%.3f, %.3f)", lin, ang);
                    } else {
                        RCLCPP_WARN(this->get_logger(),
                                    "Failed to parse: '%s'", data.c_str());
                    }
                }
                start_receive();
            }
        );
    }

    boost::asio::io_context io_;
    udp::socket socket_;
    udp::endpoint remote_endpoint_;
    std::array<char, 64> recv_buffer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    std::thread listener_thread_;

public:
    UdpToCmdVel(int port) : Node("udp_to_cmdvel"), io_(), socket_(io_, udp::endpoint(udp::v4(), port)) {
        std::stringstream ss;
        ss<<port;
        std::string port_str;
        ss>>port_str;
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), (std::string("")+"UDP listener bound to port"+port_str).c_str());
        listener_thread_ = std::thread([this]() { io_.run(); });
        start_receive();
    }

    ~UdpToCmdVel() {
        io_.stop();
        if (listener_thread_.joinable()) listener_thread_.join();
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UdpToCmdVel>(5005);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
